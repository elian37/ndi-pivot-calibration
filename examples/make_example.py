#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate the synthetic example: a fictitious 4-marker tool and a simulated pivot session.

Outputs (next to this script):
  example_tool.rom     minimal tool definition read by ndi_camera.read_rom (marker geometry only;
                       not a complete NDI file, do not load it into a camera)
  synthetic_pivot.csv  400 frames of a tool pivoting about a fixed divot, 0.1 mm noise,
                       5 % "slipped" frames. True tip offset e = (1.20, -1.80, 2.00) mm.
"""

import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import ndi_pivot  # noqa: E402

MARKERS = np.array([[-40.0, 60.0, 150.0], [40.0, 55.0, 150.0], [30.0, -45.0, 150.0], [-35.0, -60.0, 150.0]])
TIP = np.array([1.20, -1.80, 2.00])
DIVOT = np.array([380.0, -190.0, -1160.0])


def write_rom(path):
    b = bytearray(752)
    b[0:4] = b"NDI\x00"
    struct.pack_into("<i", b, 0x1C, len(MARKERS))
    for i, m in enumerate(MARKERS):
        struct.pack_into("<3f", b, 0x48 + 12 * i, *m)
    b[0x244:0x244 + 7] = b"example"
    b[0x250:0x250 + 12] = b"EXAMPLE-TOOL"
    with open(path, "wb") as f:
        f.write(b)


def rot(k, th):
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def quat(R):
    q0 = np.sqrt(1 + np.trace(R)) / 2
    return [q0, (R[2, 1] - R[1, 2]) / (4 * q0), (R[0, 2] - R[2, 0]) / (4 * q0), (R[1, 0] - R[0, 1]) / (4 * q0)]


def main():
    rng = np.random.default_rng(2026)
    u = MARKERS.mean(axis=0) - TIP
    u /= np.linalg.norm(u)
    a = np.cross(u, [1.0, 0.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(u, a)
    R0 = rot(np.array([1.0, 0.3, 0.2]), 2.6)          # shaft pointing roughly towards the camera
    rows = []
    for i in range(400):
        phi = 2 * np.pi * i / 90.0
        R = R0 @ rot(np.cos(phi) * a + np.sin(phi) * b, np.radians(25) * (0.8 + 0.2 * rng.random()))
        t = DIVOT - R @ TIP + rng.normal(0, 0.1, 3)
        if rng.random() < 0.05:
            v = rng.normal(0, 1, 3)
            t += v / np.linalg.norm(v) * rng.uniform(3, 8)
        rows.append([i] + list(np.round(quat(R), 4)) + list(np.round(t, 2)) + [0.15])
    write_rom(os.path.join(HERE, "example_tool.rom"))
    ndi_pivot.save_csv(os.path.join(HERE, "synthetic_pivot.csv"), rows)


if __name__ == "__main__":
    main()
