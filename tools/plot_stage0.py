#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage 0：把 ESP32-S3 的串口日志画成温度曲线

用法：
    # 1) 固件里 printf 改成 CSV：
    #    printf("DATA,%lld,%.2f,%.2f,%d\n", esp_timer_get_time()/1000, raw, filtered, heat);
    # 2) 存日志：
    #    idf.py -p /dev/ttyACM0 flash monitor | tee ~/log.csv
    # 3) 画图：
    #    python3 plot_log.py ~/log.csv
    #    python3 plot_log.py ~/log.csv --show     # 有图形界面时弹窗

容错设计：
    - 自动剥离 ANSI 颜色转义符（idf.py monitor 默认带颜色）
    - 只认 "DATA," 之后的字段，日志前缀（I (123) main: ...）不影响解析
    - 自动检测加热开关的跳变沿并画竖线标注
"""
import argparse
import re
import sys

import matplotlib
matplotlib.use("Agg")          # 无显示器也能跑；想弹窗加 --show
import matplotlib.pyplot as plt

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def parse(path):
    """从日志里抠出所有 DATA 行 → [(t_ms, raw, filtered, heat), ...]"""
    rows = []
    with open(path, errors="ignore") as f:
        for line in f:
            line = ANSI.sub("", line)
            if "DATA," not in line:
                continue
            payload = line.split("DATA,", 1)[1].strip()
            parts = payload.split(",")
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0])
                raw = float(parts[1])
                fil = float(parts[2])
                heat = int(float(parts[3]))
            except ValueError:
                continue      # 跳过被串口截断的残缺行
            rows.append((t, raw, fil, heat))
    return rows


def main():
    ap = argparse.ArgumentParser(description="绘制 ESP32-S3 温度曲线")
    ap.add_argument("log", help="串口日志文件（tee 存下来的那个）")
    ap.add_argument("-o", "--out", default="temp_curve.png", help="输出图片名")
    ap.add_argument("--show", action="store_true", help="弹窗显示")
    args = ap.parse_args()

    rows = parse(args.log)
    if not rows:
        print(f"❌ 在 {args.log} 里没找到 DATA 行")
        print("   检查：固件 printf 是否已改成 CSV 格式？日志是否真的存下来了？")
        sys.exit(1)

    rows.sort(key=lambda r: r[0])
    t0 = rows[0][0]
    t = [(r[0] - t0) / 1000.0 for r in rows]      # 转成秒，从 0 开始
    raw = [r[1] for r in rows]
    fil = [r[2] for r in rows]
    heat = [r[3] for r in rows]

    edges = [i for i in range(1, len(heat)) if heat[i] != heat[i - 1]]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, raw, ".", ms=4, alpha=.35, color="#999999", label="raw (1°C 分辨率)")
    ax.plot(t, fil, "-", lw=2, color="#c0392b", label="filtered (N=8)")

    ymax = max(raw + fil)
    for i in edges:
        ax.axvline(t[i], color="#2980b9", ls="--", lw=1.2, alpha=.8)
        ax.text(t[i], ymax, " 加热 ON" if heat[i] else " 加热 OFF",
                fontsize=9, color="#2980b9", va="top")

    ax.set_xlabel("time (s)")
    ax.set_ylabel("temperature (°C)")
    ax.set_title("ESP32-S3 on-chip temperature: CPU full-load heating test")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)

    print(f"✅ 已保存 {args.out}   共 {len(rows)} 个采样点，时长 {t[-1]:.0f}s")
    print(f"   滤波值 {fil[0]:.2f} → {fil[-1]:.2f} °C（{fil[-1] - fil[0]:+.2f}）")
    if edges:
        i = edges[0]
        if heat[i]:
            seg = fil[i:]
            print(f"   加热开启后 {seg[0]:.2f} → {seg[-1]:.2f} °C "
                  f"（{(t[-1] - t[i]):.0f}s 内 {seg[-1] - seg[0]:+.2f} °C）")
    print(f"   原始值台阶：{sorted(set(raw))}  ← 这就是 1°C 分辨率的证据")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
