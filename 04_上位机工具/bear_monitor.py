#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
藏马熊识别预警装置 —— PC 端 JTAG 监视器
通过 xsdb 从板卡 DDR 读取标注帧与日志，实时显示检测结果。
用法: python bear_monitor.py [--elf] [--port 3121]
"""
import os, sys, time, struct, subprocess, tempfile, argparse
import numpy as np
import cv2

VITIS = r"C:\Xilinx\Vitis\2020.2\bin"
XSDB = os.path.join(VITIS, "xsdb.bat")
ANNO = 0x0D000000
LOG = 0x0F000000
FRAME_W, FRAME_H = 320, 320
TMP = os.path.join(tempfile.gettempdir(), "bear_mon")

def dump(addr, words, path):
    tcl = os.path.join(TMP, "dump.tcl")
    with open(tcl, "w") as f:
        f.write("connect\n")
        f.write('targets -set -nocase -filter {name =~ "ARM*#0"}\n')
        f.write("mrd -force -bin -file {%s} 0x%08X %d\n" % (path.replace("\\", "/"), addr, words))
        f.write("exit\n")
    subprocess.run([XSDB, tcl], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                   env=dict(os.environ, PATH=VITIS + ";" + os.environ.get("PATH", "")))

def read_log():
    p = os.path.join(TMP, "log.bin")
    dump(LOG, 4096, p)
    try:
        d = open(p, "rb").read()
        magic, ln, seq = struct.unpack("<III", d[:12])
        if magic != 0x4A4C4F47:
            return "", 0
        return d[12:12 + ln].decode("utf-8", "replace"), seq
    except Exception:
        return "", 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=0.0)
    args = ap.parse_args()
    os.makedirs(TMP, exist_ok=True)
    img_path = os.path.join(TMP, "anno.bin")
    last = -1
    print("monitor started; press ESC/q to quit")
    while True:
        t0 = time.time()
        dump(ANNO, FRAME_W * FRAME_H * 3 // 4, img_path)
        d = np.fromfile(img_path, dtype=np.uint8)
        if d.size < FRAME_W * FRAME_H * 3:
            time.sleep(0.5); continue
        img = d[:FRAME_W * FRAME_H * 3].reshape(FRAME_H, FRAME_W, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        log, seq = read_log()
        if seq != last:
            last = seq
            tail = [l for l in log.strip().splitlines()][-3:]
            print(" | ".join(tail))
        img = cv2.resize(img, (FRAME_W * 2, FRAME_H * 2), interpolation=cv2.INTER_NEAREST)
        cv2.putText(img, "%.1f s/frame" % (time.time() - t0), (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.imshow("BEAR DETECTOR (JTAG)", img)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
