# -*- coding: utf-8 -*-
"""Minimal NDI Polaris (Vega) TCP client: connect, load a passive tool (.rom) and read poses.

Only what pivot calibration needs. Talks the NDI Combined API text protocol directly
(no NDI SDK required). Verified on a Polaris Vega XT (API revision G.003.007).
"""

import socket
import struct

DEFAULT_HOST = "169.254.8.242"
DEFAULT_PORT = 8765


# ---------------------------------------------------------------- .rom file
def read_rom(path):
    """Parse an NDI tool definition (.rom) file; returns marker geometry and identity fields.

    Little-endian layout used here: 0x1C i32 marker count, 0x48 f32x3xN marker positions,
    0x244 char[12] manufacturer, 0x250 char[20] part number.
    """
    with open(path, "rb") as f:
        b = f.read()
    if b[:4] != b"NDI\x00":
        raise ValueError("not an NDI tool definition file: %s" % path)
    n = struct.unpack_from("<i", b, 0x1C)[0]
    if not 0 < n <= 20:
        raise ValueError("unexpected marker count %d in %s" % (n, path))
    return {
        "path": path,
        "data": b,
        "n_markers": n,
        "points": [struct.unpack_from("<3f", b, 0x48 + 12 * i) for i in range(n)],
        "manufacturer": b[0x244:0x250].split(b"\x00")[0].decode("latin1"),
        "part_number": b[0x250:0x264].split(b"\x00")[0].decode("latin1"),
    }


# ---------------------------------------------------------------- connection
def _local_ipv4_addresses():
    out = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            if info[4][0] not in out:
                out.append(info[4][0])
    except socket.gaierror:
        pass
    return out


def _source_candidates(host, port):
    """Source addresses to try, in order.

    Cameras usually sit on a link-local network (169.254.0.0/16). VPN/proxy adapters
    (e.g. Tailscale when logged out) can claim the same range and capture the route, so
    for link-local targets we bind explicitly to each local link-local adapter first and
    fall back to the default route. Every candidate is retried, because virtual adapters
    may bind but never connect.
    """
    try:
        dest = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_STREAM)[0][4][0]
    except socket.gaierror:
        dest = host
    local = _local_ipv4_addresses()
    if dest.startswith("169.254."):
        return [ip for ip in local if ip.startswith("169.254.")] + [None]
    return [None] + local


class Camera(object):
    """TCP connection to the camera. Commands end with CR; replies end with CR."""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=3.0):
        last_err = None
        self.s = None
        candidates = _source_candidates(host, port)
        for src in candidates:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            try:
                if src:
                    s.bind((src, 0))
                s.connect((host, port))
                self.s = s
                self.source = src
                return
            except OSError as exc:
                last_err = exc
                s.close()
        raise OSError("无法连接相机 %s:%d（已尝试 %d 个本机地址）：%s。"
                      "若开了 VPN/代理（如 Tailscale），请先关闭或让相机网段绕过。"
                      % (host, port, len(candidates), last_err))

    def cmd(self, command, timeout=5.0):
        """Send one command, return the raw reply (latin1 text) or None on timeout/close."""
        self.s.settimeout(timeout)
        self.s.sendall((command + "\r").encode("ascii"))
        data = b""
        while not data.endswith(b"\r"):
            try:
                chunk = self.s.recv(65536)
            except socket.timeout:
                return None
            if not chunk:
                return None
            data += chunk
        return data.decode("latin1")

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def strip_crc(reply):
    """Drop the trailing CR and the 4-character CRC from a reply."""
    if reply is None:
        return "<no reply>"
    reply = reply.rstrip("\r")
    return reply[:-4] if len(reply) > 4 else reply


def is_error(reply):
    return reply is None or strip_crc(reply).startswith("ERROR")


def load_passive_tool(cam, rom_path):
    """Initialise the camera and load one passive tool; returns (port handle, part number).

    Sequence: TSTOP -> INIT -> PHRQ -> PVWR (64-byte chunks) -> PINIT -> PHINF -> PENA -> TSTART.
    Passive tools have no SROM chip, so the definition must be written on every connection.
    INIT is always sent: it clears port handles left by earlier sessions.
    """
    rom = read_rom(rom_path)
    cam.cmd("TSTOP")
    if is_error(cam.cmd("INIT")):
        raise RuntimeError("INIT failed")
    reply = cam.cmd("PHRQ *********1****")
    if is_error(reply):
        raise RuntimeError("PHRQ failed: %s" % strip_crc(reply))
    handle = reply[:2]
    data = rom["data"]
    for offset in range(0, len(data), 64):
        chunk = data[offset:offset + 64].ljust(64, b"\x00")
        reply = cam.cmd("PVWR %s%04X%s" % (handle, offset, chunk.hex()))
        if is_error(reply):
            raise RuntimeError("PVWR failed at 0x%04X: %s" % (offset, strip_crc(reply)))
    reply = cam.cmd("PINIT " + handle)
    if is_error(reply):
        raise RuntimeError("PINIT failed: %s" % strip_crc(reply))
    part = strip_crc(cam.cmd("PHINF %s0004" % handle)).strip()
    reply = cam.cmd("PENA %sD" % handle)       # WARNINGxx still enables the tool
    if is_error(reply):
        raise RuntimeError("PENA failed: %s" % strip_crc(reply))
    if is_error(cam.cmd("TSTART")):
        raise RuntimeError("TSTART failed")
    return handle, part


def parse_tx0001(raw, handle):
    """Pose of one port from a "TX 0001" reply.

    Returns (frame_number, [q0, qx, qy, qz, tx, ty, tz, rms]) or (None, None) when the tool is
    missing/disabled or the record is malformed. Translation and RMS are in mm.
    Record layout (67 chars): Q0..Qz 4x6 (1e-4), Tx..Tz 3x7 (1e-2 mm), error 6 (1e-4 mm),
    port status 8, frame number 8 (hex).
    """
    body = strip_crc(raw)
    lines = [line.rstrip("\r") for line in body.split("\n")]
    if not lines or len(lines[0]) < 4:
        return None, None
    try:
        count = int(lines[0][:2], 16)
    except ValueError:
        return None, None
    rows = [(lines[0][2:4], lines[0][4:])] + [(line[:2], line[2:]) for line in lines[1:]]
    for h, data in rows[:count]:
        if h != handle:
            continue
        if data.startswith("MISSING") or data.startswith("DISABLED") or len(data) < 67:
            return None, None
        try:
            q = [int(data[k:k + 6]) / 1e4 for k in (0, 6, 12, 18)]
            t = [int(data[k:k + 7]) / 100.0 for k in (24, 31, 38)]
            rms = int(data[45:51]) / 1e4
            frame = int(data[59:67], 16)
        except ValueError:
            return None, None
        # A real rotation quaternion has unit norm; a misaligned record practically never does.
        if abs(sum(v * v for v in q) ** 0.5 - 1.0) > 0.01:
            return None, None
        return frame, q + t + [rms]
    return None, None
