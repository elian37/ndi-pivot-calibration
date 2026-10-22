#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pivot calibration GUI for NDI optical tools (tkinter, no extra dependencies besides numpy).

Choose a .rom -> connect (live "tool recognised" status) -> adjust limits -> calibrate
(countdown, progress, live cone coverage) -> graded result. The fit is the same code as the
command line tool (ndi_pivot.compute_result). A recorded CSV can be replayed instead of a camera.
"""

import datetime
import json
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# Launched with pythonw there is no console: stdout/stderr are None and any print would raise.
if sys.stdout is None or sys.stderr is None:
    _log_dir = os.path.join(os.path.expanduser("~"), "ndi_pivot_records")
    os.makedirs(_log_dir, exist_ok=True)
    _log = open(os.path.join(_log_dir, "ndi_pivot_gui.log"), "a", encoding="utf-8", buffering=1)
    sys.stdout = sys.stdout or _log
    sys.stderr = sys.stderr or _log

import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ndi_camera as cam_io  # noqa: E402
import ndi_pivot as piv  # noqa: E402

FONT = ("Microsoft YaHei UI", 10)
FONT_BIG = ("Microsoft YaHei UI", 16, "bold")
COLOR_OK = "#1E8E3E"
COLOR_BAD = "#C5221F"
COLOR_IDLE = "#80868B"
COLOR_WARN = "#B06000"
DEFAULT_SAVE_DIR = os.path.join(os.path.expanduser("~"), "ndi_pivot_records")
SETTINGS_PATH = os.path.join(DEFAULT_SAVE_DIR, "settings.json")
LIVE_INTERVAL_S = 0.1
COVERAGE_INTERVAL_S = 1.0


def tool_name(rom_path):
    return os.path.splitext(os.path.basename(rom_path))[0]


# ---------------------------------------------------------------- data sources
class CameraSource(object):
    def __init__(self, host, port, rom_path):
        self.cam = cam_io.Camera(host, port)
        try:
            self.handle, part = cam_io.load_passive_tool(self.cam, rom_path)
        except Exception:
            self.cam.close()
            raise
        self.info = "相机 %s:%d，%s（部件号 %s）端口 %s" % (host, port, tool_name(rom_path), part, self.handle)

    def read(self):
        raw = self.cam.cmd("TX 0001", timeout=2)
        if raw is None:
            raise ConnectionError("相机无回复（网线、电源或被其他程序占用）")
        return cam_io.parse_tx0001(raw, self.handle)

    def close(self):
        try:
            self.cam.cmd("TSTOP")
        finally:
            self.cam.close()


class ReplaySource(object):
    """Replays a recorded CSV at ~60 fps in a loop; frame numbers stay unique across loops."""

    FPS = 60.0

    def __init__(self, path):
        self.rows = piv.load_csv(path)
        if not self.rows:
            raise RuntimeError("CSV 里没有数据：%s" % path)
        self.i = 0
        self.loop = 0
        self.info = "回放 %s（%d 帧，循环播放）" % (os.path.basename(path), len(self.rows))

    def read(self):
        time.sleep(1.0 / self.FPS)
        if self.i >= len(self.rows):
            self.i, self.loop = 0, self.loop + 1
        row = self.rows[self.i]
        self.i += 1
        return int(row[0]) + self.loop * 10000000, list(row[1:])

    def close(self):
        pass


# ---------------------------------------------------------------- worker thread
class Worker(threading.Thread):
    """Owns the data source; reads poses, publishes live status, collects samples. Queue only."""

    def __init__(self, factory, rom_points, out):
        super(Worker, self).__init__(daemon=True)
        self.factory, self.out = factory, out
        self.rom_points = np.asarray(rom_points, dtype=float)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.calib = None

    def start_calibration(self, seconds, delay, limits):
        now = time.time()
        with self.lock:
            self.calib = {"t_start": now + delay, "t_end": now + delay + seconds, "seconds": seconds,
                          "limits": limits, "samples": [], "reads": 0, "hits": 0,
                          "coverage": None, "t_cov": 0.0}

    def cancel_calibration(self):
        with self.lock:
            self.calib = None

    def stop(self):
        self.stop_event.set()

    def run(self):
        try:
            src = self.factory()
        except Exception as exc:
            self.out.put(("error", "连接失败：%s" % exc))
            self.out.put(("disconnected", None))
            return
        self.out.put(("connected", src.info))
        axis = piv.shaft_axis(self.rom_points)
        last_frame, last_row = None, None
        win_reads, win_hits, win_frames, t_win = 0, 0, 0, time.time()
        rate, fps, t_live = 0.0, 0.0, 0.0
        try:
            while not self.stop_event.is_set():
                try:
                    frame, row = src.read()
                except ConnectionError as exc:
                    self.out.put(("error", str(exc)))
                    break
                now = time.time()
                recognized = row is not None
                new_frame = recognized and frame != last_frame
                win_reads += 1
                win_hits += 1 if recognized else 0
                if new_frame:
                    win_frames += 1
                    last_frame, last_row = frame, row
                if now - t_win >= 1.0:
                    rate = win_hits / float(win_reads)
                    fps = win_frames / (now - t_win)
                    win_reads, win_hits, win_frames, t_win = 0, 0, 0, now

                finished = None
                with self.lock:
                    c = self.calib
                    if c is not None and now >= c["t_start"]:
                        c["reads"] += 1
                        c["hits"] += 1 if recognized else 0
                        if new_frame:
                            c["samples"].append([frame] + row)
                        if now - c["t_cov"] >= COVERAGE_INTERVAL_S and len(c["samples"]) >= 10:
                            Rs = np.array([piv.quat_to_matrix(*s[1:5]) for s in c["samples"]])
                            c["coverage"] = piv.coverage(Rs, axis)
                            c["t_cov"] = now
                        if now >= c["t_end"]:
                            finished, self.calib = c, None
                    snap = None if c is None else {
                        "phase": "countdown" if now < c["t_start"] else "collect",
                        "countdown": max(0.0, c["t_start"] - now),
                        "remaining": max(0.0, c["t_end"] - now),
                        "seconds": c["seconds"], "n": len(c["samples"]),
                        "hit_rate": c["hits"] / float(c["reads"]) if c["reads"] else 0.0,
                        "coverage": c["coverage"]}

                if finished is not None:
                    self._finish(finished)
                elif now - t_live >= LIVE_INTERVAL_S:
                    t_live = now
                    self.out.put(("live", {"recognized": recognized, "rate": rate, "fps": fps,
                                           "row": last_row if recognized else None, "calib": snap}))
                time.sleep(0.002)
        finally:
            try:
                src.close()
            except Exception:
                pass
            self.out.put(("disconnected", None))

    def _finish(self, c):
        hit_rate = c["hits"] / float(c["reads"]) if c["reads"] else 0.0
        try:
            result = piv.compute_result(c["samples"], self.rom_points, c["limits"])
            self.out.put(("done", {"result": result, "samples": c["samples"], "hit_rate": hit_rate}))
        except ValueError as exc:
            self.out.put(("done", {"error": str(exc), "samples": c["samples"], "hit_rate": hit_rate}))


# ---------------------------------------------------------------- UI
class App(object):
    LIMIT_FIELDS = [
        ("rms_max_mm", "残差 RMS 上限 (mm)", 0.1, 5.0, 0.1),
        ("rms_good_mm", "残差 RMS“好” (mm)", 0.1, 5.0, 0.1),
        ("min_minor_angle_deg", "次方向最小摆角 (°)", 5.0, 90.0, 1.0),
        ("max_split_diff_mm", "两半结果差上限 (mm)", 0.1, 5.0, 0.1),
        ("max_outlier_fraction", "离群帧比例上限", 0.05, 0.9, 0.05),
    ]

    def __init__(self, root):
        self.root = root
        self.queue = queue.Queue()
        self.worker = None
        self.connected = False
        self.calibrating = False
        self.last_offset = ""
        root.title("NDI 针尖枢轴标定")
        root.option_add("*Font", FONT)
        style = ttk.Style(root)
        style.configure(".", font=FONT)
        style.configure("Treeview", font=FONT, rowheight=24)
        style.configure("Treeview.Heading", font=(FONT[0], FONT[1], "bold"))
        self._build()
        self._load_settings()
        self._refresh_states()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self._poll)

    # ---------- layout
    def _build(self):
        pad = {"padx": 8, "pady": 4}
        main = ttk.Frame(self.root, padding=8)
        main.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(2, weight=1)

        f = ttk.LabelFrame(main, text="① 连接与工具", padding=6)
        f.grid(row=0, column=0, sticky="nsew", **pad)
        f.columnconfigure(1, weight=1)
        self.source_var = tk.StringVar(value="camera")
        ttk.Radiobutton(f, text="相机", variable=self.source_var, value="camera",
                        command=self._refresh_states).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(f, text="回放 CSV", variable=self.source_var, value="replay",
                        command=self._refresh_states).grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="相机地址").grid(row=1, column=0, sticky="w")
        self.host_var = tk.StringVar(value=cam_io.DEFAULT_HOST)
        self.port_var = tk.StringVar(value=str(cam_io.DEFAULT_PORT))
        self.host_entry = ttk.Entry(f, textvariable=self.host_var, width=16)
        self.host_entry.grid(row=1, column=1, sticky="w")
        self.port_entry = ttk.Entry(f, textvariable=self.port_var, width=6)
        self.port_entry.grid(row=1, column=2, sticky="w")
        ttk.Label(f, text="工具 .rom").grid(row=2, column=0, sticky="w")
        self.rom_var = tk.StringVar()
        self.rom_entry = ttk.Entry(f, textvariable=self.rom_var, width=24)
        self.rom_entry.grid(row=2, column=1, columnspan=2, sticky="we")
        self.rom_btn = ttk.Button(f, text="浏览…", command=self._browse_rom)
        self.rom_btn.grid(row=2, column=3, sticky="w")
        ttk.Label(f, text="回放文件").grid(row=3, column=0, sticky="w")
        self.csv_var = tk.StringVar()
        self.csv_entry = ttk.Entry(f, textvariable=self.csv_var, width=24)
        self.csv_entry.grid(row=3, column=1, columnspan=2, sticky="we")
        self.csv_btn = ttk.Button(f, text="浏览…", command=self._browse_csv)
        self.csv_btn.grid(row=3, column=3, sticky="w")
        self.connect_btn = ttk.Button(f, text="连接", command=self.on_connect)
        self.connect_btn.grid(row=4, column=1, sticky="we", pady=(6, 0))
        self.disconnect_btn = ttk.Button(f, text="断开", command=self.on_disconnect)
        self.disconnect_btn.grid(row=4, column=2, columnspan=2, sticky="we", pady=(6, 0))
        self.conn_label = ttk.Label(f, text="未连接", foreground=COLOR_IDLE, wraplength=380)
        self.conn_label.grid(row=5, column=0, columnspan=4, sticky="w", pady=(4, 0))

        f = ttk.LabelFrame(main, text="② 实时状态", padding=6)
        f.grid(row=0, column=1, sticky="nsew", **pad)
        self.recog_label = ttk.Label(f, text="● 未连接", font=FONT_BIG, foreground=COLOR_IDLE)
        self.recog_label.grid(row=0, column=0, columnspan=2, sticky="w")
        self.live_vars = {}
        for i, (key, text) in enumerate([("rate", "识别率（近 1 秒）"), ("fps", "数据帧率"),
                                         ("rms", "工具 RMS"), ("pos", "工具原点位置 (mm)")]):
            ttk.Label(f, text=text).grid(row=i + 1, column=0, sticky="w")
            v = tk.StringVar(value="—")
            ttk.Label(f, textvariable=v).grid(row=i + 1, column=1, sticky="w")
            self.live_vars[key] = v
        ttk.Label(f, text="提示：未识别时检查球是否被手挡住、是否朝向相机、是否在视野中央约 1 m 处。",
                  foreground=COLOR_IDLE, wraplength=360).grid(row=5, column=0, columnspan=2, sticky="w", pady=(4, 0))

        f = ttk.LabelFrame(main, text="③ 标定参数", padding=6)
        f.grid(row=1, column=0, sticky="nsew", **pad)
        self.seconds_var = tk.IntVar(value=40)
        self.delay_var = tk.IntVar(value=3)
        self.param_widgets = []
        ttk.Label(f, text="采集时长 (s)").grid(row=0, column=0, sticky="w")
        w = ttk.Spinbox(f, from_=10, to=180, increment=5, textvariable=self.seconds_var, width=8)
        w.grid(row=0, column=1, sticky="w"); self.param_widgets.append(w)
        ttk.Label(f, text="开始前倒计时 (s)").grid(row=1, column=0, sticky="w")
        w = ttk.Spinbox(f, from_=0, to=10, increment=1, textvariable=self.delay_var, width=8)
        w.grid(row=1, column=1, sticky="w"); self.param_widgets.append(w)
        self.limit_vars = {}
        for i, (key, text, lo, hi, step) in enumerate(self.LIMIT_FIELDS):
            ttk.Label(f, text=text).grid(row=i + 2, column=0, sticky="w")
            v = tk.DoubleVar(value=piv.DEFAULT_LIMITS[key])
            w = ttk.Spinbox(f, from_=lo, to=hi, increment=step, textvariable=v, width=8)
            w.grid(row=i + 2, column=1, sticky="w")
            self.limit_vars[key] = v
            self.param_widgets.append(w)
        w = ttk.Button(f, text="恢复默认", command=self._reset_params)
        w.grid(row=0, column=2, sticky="e"); self.param_widgets.append(w)
        self.save_var = tk.BooleanVar(value=True)
        w = ttk.Checkbutton(f, text="自动保存原始数据到", variable=self.save_var)
        w.grid(row=7, column=0, sticky="w", pady=(6, 0)); self.param_widgets.append(w)
        self.save_dir_var = tk.StringVar(value=DEFAULT_SAVE_DIR)
        w = ttk.Entry(f, textvariable=self.save_dir_var, width=26)
        w.grid(row=7, column=1, columnspan=2, sticky="we", pady=(6, 0)); self.param_widgets.append(w)

        f = ttk.LabelFrame(main, text="④ 标定", padding=6)
        f.grid(row=1, column=1, sticky="nsew", **pad)
        self.start_btn = ttk.Button(f, text="开始标定", command=self.on_start)
        self.start_btn.grid(row=0, column=0, sticky="we")
        self.cancel_btn = ttk.Button(f, text="取消", command=self.on_cancel)
        self.cancel_btn.grid(row=0, column=1, sticky="we")
        self.phase_label = ttk.Label(f, text="连接并识别到工具后，点“开始标定”。", font=FONT_BIG,
                                     foreground=COLOR_IDLE, wraplength=380)
        self.phase_label.grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 2))
        self.progress = ttk.Progressbar(f, mode="determinate", maximum=100, length=360)
        self.progress.grid(row=2, column=0, columnspan=2, sticky="we")
        self.calib_vars = {}
        for i, (key, text) in enumerate([("n", "已采有效帧"), ("hit", "采集中识别率"),
                                         ("minor", "次方向摆角（需画圆锥）"), ("major", "主方向摆角")]):
            ttk.Label(f, text=text).grid(row=i + 3, column=0, sticky="w")
            v = tk.StringVar(value="—")
            lbl = ttk.Label(f, textvariable=v)
            lbl.grid(row=i + 3, column=1, sticky="w")
            self.calib_vars[key] = (v, lbl)
        ttk.Label(f, text="操作：针尖放进凹坑（凹坑需能卡住针尖），手握杆上端，倾斜 30°~45° 绕针尖画圆锥。",
                  foreground=COLOR_IDLE, wraplength=380).grid(row=7, column=0, columnspan=2, sticky="w", pady=(4, 0))

        f = ttk.LabelFrame(main, text="⑤ 结果", padding=6)
        f.grid(row=2, column=0, columnspan=2, sticky="nsew", **pad)
        f.columnconfigure(0, weight=1)
        f.rowconfigure(3, weight=1)
        self.verdict_label = ttk.Label(f, text="尚未标定", font=FONT_BIG, foreground=COLOR_IDLE)
        self.verdict_label.grid(row=0, column=0, sticky="w")
        self.copy_btn = ttk.Button(f, text="复制针尖偏移", command=self.on_copy)
        self.copy_btn.grid(row=0, column=1, sticky="e")
        self.summary_text = tk.Text(f, height=3, wrap="word", font=FONT, relief="flat",
                                    background=self.root.cget("background"))
        self.summary_text.grid(row=1, column=0, columnspan=2, sticky="we", pady=(4, 4))
        self.summary_text.tag_configure("head", font=(FONT[0], FONT[1], "bold"))
        self.summary_text.tag_configure("bad", foreground=COLOR_BAD, font=(FONT[0], FONT[1], "bold"))
        cols = [("name", "检查项", 180), ("value", "结果", 110), ("std", "标准", 360), ("verdict", "判定", 80)]
        self.checks_tree = ttk.Treeview(f, columns=[c[0] for c in cols], show="headings", height=6)
        for key, text, width in cols:
            self.checks_tree.heading(key, text=text)
            self.checks_tree.column(key, width=width, anchor="w", stretch=(key == "std"))
        self.checks_tree.tag_configure("ok", foreground=COLOR_OK)
        self.checks_tree.tag_configure("bad", foreground=COLOR_BAD)
        self.checks_tree.grid(row=2, column=0, columnspan=2, sticky="we")
        self.note_text = tk.Text(f, height=4, wrap="none", font=("Consolas", 10))
        self.note_text.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(4, 0))
        self.note_text.tag_configure("bad", foreground=COLOR_BAD, font=("Consolas", 10, "bold"))
        for w in (self.summary_text, self.note_text):
            w.configure(state="disabled")

    # ---------- settings
    def _load_settings(self):
        try:
            with open(SETTINGS_PATH, encoding="utf-8") as f:
                s = json.load(f)
        except (OSError, ValueError):
            return
        self.host_var.set(s.get("host", self.host_var.get()))
        self.port_var.set(str(s.get("port", self.port_var.get())))
        self.rom_var.set(s.get("rom", ""))

    def _save_settings(self):
        try:
            os.makedirs(os.path.dirname(SETTINGS_PATH), exist_ok=True)
            with open(SETTINGS_PATH, "w", encoding="utf-8") as f:
                json.dump({"host": self.host_var.get(), "port": self.port_var.get(),
                           "rom": self.rom_var.get()}, f, ensure_ascii=False)
        except OSError:
            pass

    # ---------- state
    def _refresh_states(self):
        idle = not self.connected and self.worker is None
        camera = self.source_var.get() == "camera"
        for w in (self.host_entry, self.port_entry):
            w.configure(state="normal" if idle and camera else "disabled")
        for w in (self.csv_entry, self.csv_btn):
            w.configure(state="normal" if idle and not camera else "disabled")
        for w in (self.rom_entry, self.rom_btn, self.connect_btn):
            w.configure(state="normal" if idle else "disabled")
        self.disconnect_btn.configure(state="normal" if self.worker is not None else "disabled")
        self.start_btn.configure(state="normal" if self.connected and not self.calibrating else "disabled")
        self.cancel_btn.configure(state="normal" if self.calibrating else "disabled")
        for w in self.param_widgets:
            w.configure(state="disabled" if self.calibrating else "normal")
        self.copy_btn.configure(state="normal" if self.last_offset else "disabled")

    def _reset_params(self):
        self.seconds_var.set(40)
        self.delay_var.set(3)
        for key, v in self.limit_vars.items():
            v.set(piv.DEFAULT_LIMITS[key])

    def _browse_rom(self):
        path = filedialog.askopenfilename(title="选择工具定义文件", filetypes=[("NDI tool", "*.rom"), ("全部", "*.*")])
        if path:
            self.rom_var.set(path)

    def _browse_csv(self):
        path = filedialog.askopenfilename(title="选择回放数据", filetypes=[("CSV", "*.csv"), ("全部", "*.*")])
        if path:
            self.csv_var.set(path)

    # ---------- buttons
    def on_connect(self):
        rom = self.rom_var.get().strip()
        try:
            rom_points = cam_io.read_rom(rom)["points"]
        except (OSError, ValueError) as exc:
            messagebox.showwarning("连接", "请选择有效的 .rom 文件：%s" % exc)
            return
        if self.source_var.get() == "camera":
            try:
                host, port = self.host_var.get().strip(), int(self.port_var.get())
            except ValueError:
                messagebox.showwarning("连接", "端口必须是数字。")
                return
            factory = lambda: CameraSource(host, port, rom)  # noqa: E731
            self.conn_label.configure(text="正在连接相机并加载 %s…" % tool_name(rom), foreground=COLOR_WARN)
        else:
            path = self.csv_var.get().strip()
            if not os.path.isfile(path):
                messagebox.showwarning("连接", "请选择存在的回放 CSV 文件。")
                return
            factory = lambda: ReplaySource(path)  # noqa: E731
            self.conn_label.configure(text="正在加载回放数据…", foreground=COLOR_WARN)
        self._save_settings()
        self.worker = Worker(factory, rom_points, self.queue)
        self.worker.start()
        self._refresh_states()

    def on_disconnect(self):
        if self.worker is not None:
            self.worker.stop()
        self.conn_label.configure(text="正在断开…", foreground=COLOR_WARN)

    def on_start(self):
        try:
            seconds, delay = int(self.seconds_var.get()), int(self.delay_var.get())
            limits = {k: float(v.get()) for k, v in self.limit_vars.items()}
        except (tk.TclError, ValueError):
            messagebox.showwarning("参数", "参数必须是数字。")
            return
        if seconds < 5:
            messagebox.showwarning("参数", "采集时长至少 5 秒。")
            return
        self.calibrating = True
        self.progress.configure(value=0)
        self.worker.start_calibration(seconds, delay, limits)
        self._refresh_states()

    def on_cancel(self):
        if self.worker is not None:
            self.worker.cancel_calibration()
        self._end_calibration("已取消标定。", COLOR_IDLE)

    def on_copy(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.last_offset)
        self.verdict_label.configure(text=self.verdict_label.cget("text").split("（")[0] + "（已复制）")

    def on_close(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(timeout=6)
        self.root.destroy()

    def _end_calibration(self, text, color):
        self.calibrating = False
        self.phase_label.configure(text=text, foreground=color)
        self._refresh_states()

    # ---------- queue
    def _poll(self):
        try:
            while True:
                kind, data = self.queue.get_nowait()
                getattr(self, "_on_" + kind)(data)
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def _on_connected(self, info):
        self.connected = True
        self.conn_label.configure(text="已连接：" + info, foreground=COLOR_OK)
        self._refresh_states()

    def _on_disconnected(self, _):
        self.connected = False
        self.worker = None
        if self.calibrating:
            self._end_calibration("连接断开，标定中止。", COLOR_BAD)
        if "失败" not in self.conn_label.cget("text") and "无回复" not in self.conn_label.cget("text"):
            self.conn_label.configure(text="未连接（相机已释放）", foreground=COLOR_IDLE)
        self.recog_label.configure(text="● 未连接", foreground=COLOR_IDLE)
        for v in self.live_vars.values():
            v.set("—")
        self._refresh_states()

    def _on_error(self, msg):
        self.conn_label.configure(text=msg, foreground=COLOR_BAD)

    def _on_live(self, d):
        if d["recognized"]:
            self.recog_label.configure(text="● 已识别到工具", foreground=COLOR_OK)
        else:
            self.recog_label.configure(text="● 未识别（看不到工具）", foreground=COLOR_BAD)
        self.live_vars["rate"].set("%.0f%%" % (100 * d["rate"]))
        self.live_vars["fps"].set("%.0f 帧/秒" % d["fps"])
        row = d["row"]
        if row is not None:
            self.live_vars["rms"].set("%.3f mm" % row[7])
            self.live_vars["pos"].set("%.1f, %.1f, %.1f" % (row[4], row[5], row[6]))
        else:
            self.live_vars["rms"].set("—")
            self.live_vars["pos"].set("—")
        c = d["calib"]
        if c is None or not self.calibrating:
            return
        if c["phase"] == "countdown":
            self.phase_label.configure(text="%.0f 秒后开始：针尖放进凹坑，准备转动" % (c["countdown"] + 0.49),
                                       foreground=COLOR_WARN)
            self.progress.configure(value=0)
        else:
            self.phase_label.configure(text="采集中：绕针尖画圆锥，剩余 %.0f 秒" % (c["remaining"] + 0.49),
                                       foreground=COLOR_OK)
            self.progress.configure(value=100.0 * (1 - c["remaining"] / c["seconds"]))
        self.calib_vars["n"][0].set(str(c["n"]))
        self.calib_vars["hit"][0].set("%.0f%%" % (100 * c["hit_rate"]))
        if c["coverage"] is not None:
            major, minor = c["coverage"]
            need = float(self.limit_vars["min_minor_angle_deg"].get())
            self.calib_vars["minor"][0].set("%.1f°（需 ≥ %.0f°）" % (minor, need))
            self.calib_vars["minor"][1].configure(foreground=COLOR_OK if minor >= need else COLOR_BAD)
            self.calib_vars["major"][0].set("%.1f°" % major)

    def _on_done(self, d):
        self.progress.configure(value=100)
        saved = ""
        if self.save_var.get() and d["samples"]:
            try:
                folder = self.save_dir_var.get().strip() or DEFAULT_SAVE_DIR
                stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                path = os.path.join(folder, "%s_%s.csv" % (tool_name(self.rom_var.get()), stamp))
                piv.save_csv(path, d["samples"])
                saved = path
            except OSError as exc:
                saved = "保存失败：%s" % exc
        notes, summary = [], []
        self.checks_tree.delete(*self.checks_tree.get_children())
        if "error" in d:
            self.last_offset = ""
            self.verdict_label.configure(text="✗ 无法计算", foreground=COLOR_BAD)
            summary = [(d["error"] + "\n", "bad"), ("采集中识别率 %.0f%%。\n" % (100 * d["hit_rate"]), None)]
            self._end_calibration("采集结束，但数据不足。", COLOR_BAD)
        else:
            r = d["result"]
            e = r["e"]
            summary = self._summary_chunks(r, d["hit_rate"])
            for name, val, std, ok in r["checks"]:
                self.checks_tree.insert("", "end", values=(name, val, std, "通过" if ok else "不通过"),
                                        tags=("ok" if ok else "bad",))
            if r["passed"]:
                self.last_offset = "%.3f, %.3f, %.3f" % (e[0], e[1], e[2])
                notes = [("针尖偏移（.rom 坐标系，mm）：%s\n建议再做一次，两次相差 ≤ 0.5 mm 再采用。\n"
                          % self.last_offset, None)]
                self.verdict_label.configure(text="✓ 达标" + ("（结果好）" if r["rms_good"] else ""),
                                             foreground=COLOR_OK)
                self._end_calibration("标定完成：达标。建议再做一次核对。", COLOR_OK)
            else:
                self.last_offset = ""
                notes = [("未达标，不要采用：按红色项调整（凹坑卡住针尖、画圆锥、慢、手不挡球）后重做。\n", "bad")]
                self.verdict_label.configure(text="✗ 未达标，不要采用", foreground=COLOR_BAD)
                self._end_calibration("标定完成：未达标，按红色项调整后重做。", COLOR_BAD)
        if saved:
            notes.append(("原始数据：%s\n" % saved, None))
        self._set_text(self.summary_text, summary)
        self._set_text(self.note_text, notes)
        self._refresh_states()

    @staticmethod
    def _set_text(widget, chunks):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        for text, tag in chunks:
            if tag:
                widget.insert("end", text, tag)
            else:
                widget.insert("end", text)
        widget.configure(state="disabled")

    @staticmethod
    def _summary_chunks(r, hit_rate):
        e, se = r["e"], r["se"]
        return [("针尖偏移 e（.rom 坐标系）：x=%.3f  y=%.3f  z=%.3f mm    |e| = %.3f mm    （1σ ±%.3f ±%.3f ±%.3f）\n"
                 % (e[0], e[1], e[2], float(np.linalg.norm(e)), se[0], se[1], se[2]), "head"),
                ("有效帧 %d，剔除离群 %d（%.0f%%），采集中识别率 %.0f%%；工具 RMS 均值 %.3f mm\n"
                 % (r["n_all"], r["n_all"] - r["n_in"], 100 * r["outlier_frac"], 100 * hit_rate,
                    r["tool_rms_mean"]), None)]


def main():
    root = tk.Tk()
    App(root)
    root.geometry("1000x940")
    root.minsize(900, 740)
    root.mainloop()


if __name__ == "__main__":
    main()
