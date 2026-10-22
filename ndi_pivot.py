#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robust pivot calibration for NDI optical tools (command line).

Pivot model: the tip rests in a fixed divot while the tool is rotated, so for every frame i
    R_i * e + t_i = p
e = tip position in the tool (.rom) frame, p = divot position in the camera frame.
Solved by linear least squares, then frames that do not fit (tip slipped, marker occluded)
are removed iteratively (median + 3 * 1.4826 * MAD, floor 0.3 mm) and the fit is repeated.
The result is graded on residual RMS, cone coverage, split-half repeatability and outlier
fraction, so a bad session is reported as bad instead of silently producing a number.

Usage:
  python ndi_pivot.py --rom my_probe.rom                    # connect, count down, record 40 s
  python ndi_pivot.py --rom my_probe.rom --save run1.csv    # keep raw poses
  python ndi_pivot.py --rom my_probe.rom --load run1.csv    # offline re-analysis, no camera
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

import ndi_camera as cam_io

# Grading limits (all editable in the GUI).
DEFAULT_LIMITS = {
    "rms_good_mm": 0.5,           # residual RMS considered "good"
    "rms_max_mm": 1.0,            # residual RMS upper limit
    "min_minor_angle_deg": 30.0,  # minor swing range: must draw a cone, not swing in one plane
    "max_split_diff_mm": 0.5,     # first-half vs second-half tip estimate
    "max_outlier_fraction": 0.2,  # too many rejected frames means the tip kept slipping
}
MIN_FRAMES = 50
OUTLIER_FLOOR_MM = 0.3   # never treat sub-0.3 mm residuals as outliers
CSV_HEADER = ["frame", "q0", "qx", "qy", "qz", "tx", "ty", "tz", "err"]


def quat_to_matrix(q0, qx, qy, qz):
    """Rotation matrix from a unit quaternion (q0 = scalar part, NDI order)."""
    n = (q0 * q0 + qx * qx + qy * qy + qz * qz) ** 0.5
    q0, qx, qy, qz = q0 / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * q0), 2 * (qx * qz + qy * q0)],
        [2 * (qx * qy + qz * q0), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * q0)],
        [2 * (qx * qz - qy * q0), 2 * (qy * qz + qx * q0), 1 - 2 * (qx * qx + qy * qy)],
    ])


def save_csv(path, samples):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(samples)


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        next(r)
        return [[float(v) for v in row] for row in r]


def fit(Rs, Ts):
    """Least-squares pivot: [R_i  -I] [e; p] = -t_i. Returns e, p, per-frame residual, covariance."""
    n = len(Rs)
    A = np.zeros((3 * n, 6))
    for i, R in enumerate(Rs):
        A[3 * i:3 * i + 3, :3] = R
        A[3 * i:3 * i + 3, 3:] = -np.eye(3)
    y = -Ts.reshape(-1)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    e, p = x[:3], x[3:]
    res = np.linalg.norm(np.einsum("nij,j->ni", Rs, e) + Ts - p, axis=1)
    sigma2 = float(np.sum((A @ x - y) ** 2)) / max(3 * n - 6, 1)
    return e, p, res, sigma2 * np.linalg.pinv(A.T @ A)


def robust_fit(Rs, Ts):
    inliers = np.ones(len(Rs), dtype=bool)
    for _ in range(20):
        e, p, _, _ = fit(Rs[inliers], Ts[inliers])
        res = np.linalg.norm(np.einsum("nij,j->ni", Rs, e) + Ts - p, axis=1)
        med = float(np.median(res[inliers]))
        mad = float(np.median(np.abs(res[inliers] - med)))
        keep = res <= max(med + 3 * 1.4826 * mad, OUTLIER_FLOOR_MM)
        if keep.sum() < 6 or np.array_equal(keep, inliers):
            break
        inliers = keep
    e, p, res_in, cov = fit(Rs[inliers], Ts[inliers])
    return e, p, inliers, res_in, cov


def coverage(Rs, axis_local):
    """Swing range of the tool axis in the camera frame.

    axis_local: a direction fixed in the tool frame, normally tip -> marker centroid (the shaft).
    Returns (major range deg, minor range deg). A small minor range means the tool was swung in
    one plane instead of around a cone, which leaves the tip offset poorly constrained.
    """
    u = np.asarray(axis_local, dtype=float)
    u = u / np.linalg.norm(u)
    d = np.einsum("nij,j->ni", Rs, u)
    d /= np.linalg.norm(d, axis=1)[:, None]
    m = d.mean(axis=0)
    m /= np.linalg.norm(m)
    a = np.cross(m, [1.0, 0.0, 0.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(m, [0.0, 1.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(m, a)
    ang = np.degrees(np.column_stack([np.arctan2(d @ a, d @ m), np.arctan2(d @ b, d @ m)]))
    ang -= ang.mean(axis=0)
    _, vecs = np.linalg.eigh(np.cov(ang.T))
    ext = np.ptp(ang @ vecs, axis=0)       # column 0 = minor, column 1 = major
    return float(ext[1]), float(ext[0])


def shaft_axis(rom_points, tip=(0.0, 0.0, 0.0)):
    return np.mean(np.asarray(rom_points, dtype=float), axis=0) - np.asarray(tip, dtype=float)


def compute_result(samples, rom_points, limits=None):
    """Fit and grade one session. Raises ValueError when there are too few frames."""
    lim = dict(DEFAULT_LIMITS)
    lim.update(limits or {})
    S = np.array(samples, dtype=float)
    n_all = len(S)
    if n_all < MIN_FRAMES:
        raise ValueError("有效帧只有 %d，少于 %d，无法可靠拟合。检查工具是否在视野内、球是否被挡。"
                         % (n_all, MIN_FRAMES))
    Rs = np.array([quat_to_matrix(*row[1:5]) for row in S])
    Ts = S[:, 5:8]

    e, p, inl, res_in, cov = robust_fit(Rs, Ts)
    n_in = int(inl.sum())
    rms = float(np.sqrt(np.mean(res_in ** 2)))
    res_max = float(res_in.max())
    idx = np.flatnonzero(inl)
    half = len(idx) // 2
    split_diff = float(np.linalg.norm(fit(Rs[idx[:half]], Ts[idx[:half]])[0]
                                      - fit(Rs[idx[half:]], Ts[idx[half:]])[0]))
    major, minor = coverage(Rs[inl], shaft_axis(rom_points, e))
    outlier_frac = 1.0 - n_in / n_all
    raw_spread = np.linalg.norm(Ts - Ts.mean(axis=0), axis=1)

    checks = [
        ("拟合残差 RMS", "%.3f mm" % rms,
         "≤ %.1f mm（≤ %.1f 为好）" % (lim["rms_max_mm"], lim["rms_good_mm"]), rms <= lim["rms_max_mm"]),
        ("拟合残差最大值", "%.3f mm" % res_max, "与 RMS 同量级（≤ 4×RMS）", res_max <= 4 * max(rms, 0.1)),
        ("次方向摆角范围", "%.1f°" % minor,
         "≥ %.0f°（画圆锥，而非单平面来回摆）" % lim["min_minor_angle_deg"], minor >= lim["min_minor_angle_deg"]),
        ("主方向摆角范围", "%.1f°" % major, "参考", True),
        ("前后两半分别拟合之差", "%.3f mm" % split_diff,
         "≤ %.1f mm（结果可复现）" % lim["max_split_diff_mm"], split_diff <= lim["max_split_diff_mm"]),
        ("离群帧比例", "%.0f%%" % (100 * outlier_frac),
         "≤ %.0f%%（太多说明针尖经常滑）" % (100 * lim["max_outlier_fraction"]),
         outlier_frac <= lim["max_outlier_fraction"]),
    ]
    return {
        "n_all": n_all, "n_in": n_in, "outlier_frac": outlier_frac,
        "tool_rms_mean": float(S[:, 8].mean()),
        "raw_spread_rms": float(np.sqrt(np.mean(raw_spread ** 2))),
        "e": e, "se": np.sqrt(np.clip(np.diag(cov)[:3], 0, None)), "p": p,
        "rms": rms, "res_max": res_max, "minor": minor, "major": major, "split_diff": split_diff,
        "checks": checks,
        "passed": all(ok for _, _, _, ok in checks),
        "rms_good": rms <= lim["rms_good_mm"],
    }


def format_result(r):
    e, se, p = r["e"], r["se"], r["p"]
    out = ["",
           "样本：有效帧 %d，剔除离群 %d（%.0f%%），参与拟合 %d"
           % (r["n_all"], r["n_all"] - r["n_in"], 100 * r["outlier_frac"], r["n_in"]),
           "工具 RMS 均值 %.3f mm（.rom 几何与实物的吻合度，与针尖无关）" % r["tool_rms_mean"],
           "未校正时工具原点随转动的离散：RMS %.3f mm" % r["raw_spread_rms"],
           "",
           "针尖偏移 e（.rom 坐标系，mm）：x=%.3f  y=%.3f  z=%.3f   |e|=%.3f" % (e[0], e[1], e[2], np.linalg.norm(e)),
           "  统计误差（1σ）：±%.3f  ±%.3f  ±%.3f" % tuple(se),
           "凹坑位置 p（相机坐标系，mm）：%.2f  %.2f  %.2f" % tuple(p),
           "",
           "%-14s %-12s %-34s %s" % ("检查项", "结果", "标准", "判定")]
    for name, val, std, ok in r["checks"]:
        out.append("%-14s %-12s %-34s %s" % (name, val, std, "通过" if ok else "不通过"))
    out.append("")
    if r["passed"]:
        out.append("结论：达标。建议再做一次，两次 e 相差 ≤ 0.5 mm 再采用。")
    else:
        out.append("结论：未达标，不要采用。按「不通过」项调整后重做（凹坑卡住针尖、画圆锥、慢、手不挡球）。")
    return out


def capture(args):
    cam = cam_io.Camera(args.host, args.port)
    samples = []
    try:
        handle, part = cam_io.load_passive_tool(cam, args.rom)
        print("已连接 %s:%d，工具 %s（部件号 %s）在端口 %s" % (args.host, args.port,
              os.path.basename(args.rom), part, handle))
        for k in range(args.delay, 0, -1):
            print("  %d 秒后开始采集：针尖放进凹坑，准备转动……" % k)
            time.sleep(1)
        print(">>> 开始采集 %d 秒：绕针尖缓慢画圆锥（倾斜 30°~45°，前后左右都转到）" % args.seconds)
        start = time.time()
        last_frame, missing, next_report = None, 0, 1.0
        while time.time() - start < args.seconds:
            raw = cam.cmd("TX 0001", timeout=2)
            if raw is None:
                print("  相机无回复，停止采集。")
                break
            frame, row = cam_io.parse_tx0001(raw, handle)
            if row is None:
                missing += 1
            elif frame != last_frame:
                samples.append([frame] + row)
                last_frame = frame
            if time.time() - start >= next_report:
                print("  %4.0f s  有效帧 %d  未识别 %d" % (time.time() - start, len(samples), missing))
                next_report += 1.0
        print("<<< 采集结束：有效帧 %d，未识别 %d。" % (len(samples), missing))
        cam.cmd("TSTOP")
    finally:
        cam.close()
    return samples


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description="NDI 光学工具针尖枢轴标定（稳健拟合）")
    ap.add_argument("--rom", required=True, help="工具定义文件（.rom）路径")
    ap.add_argument("--host", default=cam_io.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=cam_io.DEFAULT_PORT)
    ap.add_argument("--seconds", type=int, default=40, help="采集时长（秒），默认 40")
    ap.add_argument("--delay", type=int, default=3, help="开始前倒计时（秒），默认 3")
    ap.add_argument("--save", help="把原始位姿保存为 CSV，便于离线复算")
    ap.add_argument("--load", help="不连相机，读取 --save 存下的 CSV 离线重算")
    args = ap.parse_args()

    rom_points = cam_io.read_rom(args.rom)["points"]
    if args.load:
        samples = load_csv(args.load)
    else:
        samples = capture(args)
        if args.save:
            save_csv(args.save, samples)
            print("原始数据已保存：%s" % args.save)
    try:
        r = compute_result(samples, rom_points)
    except ValueError as exc:
        print(exc)
        return 2
    print("\n".join(format_result(r)))
    return 0 if r["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
