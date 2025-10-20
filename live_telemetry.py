#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, re, sys, time, json
from collections import deque
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def parse_line(line: str):
    """
    兼容三种格式：
    1) key=value 用逗号分隔  t=...,roll=...,ax=...,x=...,y=...,z=...
    2) 纯 CSV，按固定顺序: t,ax,ay,az,gx,gy,gz,roll,pitch,yaw,x,y,z
    3) JSON: {"t":..,"roll":.., ...}
    """
    line = line.strip()
    if not line:
        return {}

    # JSON
    if line.startswith("{") and line.endswith("}"):
        try:
            return json.loads(line)
        except Exception:
            return {}

    # key=value
    if "=" in line and "," in line:
        out = {}
        for kv in line.split(","):
            if "=" in kv:
                k,v = kv.split("=",1)
                try:
                    out[k.strip()] = float(v)
                except ValueError:
                    out[k.strip()] = float("nan")
        return out

    # CSV fallback
    try:
        vals = [float(x) for x in line.split(",")]
        keys = ["t","ax","ay","az","gx","gy","gz","roll","pitch","yaw","x","y","z"]
        return {k:v for k,v in zip(keys, vals)}
    except Exception:
        return {}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--buf",  type=int, default=400, help="滚动缓存长度")
    ap.add_argument("--save", type=str, default="", help="保存到CSV文件（可选）")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    print(f"[INFO] Open {ser.port} @ {ser.baudrate}")

    # 滚动窗口
    t  = deque(maxlen=args.buf)
    X  = deque(maxlen=args.buf)
    Y  = deque(maxlen=args.buf)
    Z  = deque(maxlen=args.buf)

    XYx = deque(maxlen=args.buf)
    XYy = deque(maxlen=args.buf)

    # 画布：上面时序曲线（X/Y/Z），下面平面视图（优先 x/y 轨迹；缺失则用 roll/pitch 映射）
    plt.figure(figsize=(10,7))
    ax1 = plt.subplot(2,1,1)
    ax2 = plt.subplot(2,1,2)

    lineX, = ax1.plot([], [], label="X")
    lineY, = ax1.plot([], [], label="Y")
    lineZ, = ax1.plot([], [], label="Z")
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("value")
    ax1.legend(loc="upper right")
    ax1.grid(True)

    scat = ax2.plot([], [], marker="o", linestyle="-", markersize=3)[0]
    ax2.set_aspect("equal", adjustable="box")
    ax2.set_xlabel("X (m or au)")
    ax2.set_ylabel("Y (m or au)")
    ax2.grid(True)
    ax2.set_title("XY 平面轨迹（若缺失位置则用 roll/pitch 投影）")

    start_t = None
    fout = open(args.save, "w", buffering=1) if args.save else None
    if fout:
        fout.write("t,roll,pitch,yaw,ax,ay,az,x,y,z\n")

    def update(_):
        nonlocal start_t
        try:
            line = ser.readline().decode(errors="ignore")
        except Exception:
            return lineX, lineY, lineZ, scat

        d = parse_line(line)
        if not d:
            return lineX, lineY, lineZ, scat

        now_t = d.get("t",None)
        if start_t is None:
            start_t = now_t if now_t is not None else time.time()*1000.0

        # 时间（秒）
        if now_t is None:
            ts = time.time()*1000.0 - start_t
        else:
            ts = now_t - start_t
        t.append(ts/1000.0)

        # 选取 X/Y/Z 序列：优先位置 x,y,z；若 NaN 或缺失，回退到 ax,ay,az
        def pick(key, alt):
            v = d.get(key, float("nan"))
            if np.isnan(v):
                v = d.get(alt, float("nan"))
            return v

        x = pick("x", "ax")
        y = pick("y", "ay")
        z = pick("z", "az")
        X.append(x); Y.append(y); Z.append(z)

        # 平面：优先 x/y，否则用 roll/pitch 的切线近似投影
        if not np.isnan(d.get("x", float("nan"))) and not np.isnan(d.get("y", float("nan"))):
            XYx.append(d["x"]); XYy.append(d["y"])
        else:
            roll  = np.deg2rad(d.get("roll", 0.0))
            pitch = np.deg2rad(d.get("pitch",0.0))
            XYx.append(np.tan(roll))   # 小角度近似
            XYy.append(np.tan(pitch))

        # 写文件
        if fout:
            fout.write("{:.3f},{:.3f},{:.3f},{:.3f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n".format(
                t[-1], d.get("roll",np.nan), d.get("pitch",np.nan), d.get("yaw",np.nan),
                d.get("ax",np.nan), d.get("ay",np.nan), d.get("az",np.nan),
                d.get("x",np.nan),  d.get("y",np.nan),  d.get("z",np.nan),
            ))

        # 更新曲线
        lineX.set_data(t, X)
        lineY.set_data(t, Y)
        lineZ.set_data(t, Z)
        # 自适应 x 轴范围
        if len(t) > 5:
            ax1.set_xlim(t[0], t[-1])
        # y 轴范围自动：取最近窗口的 5% 余量
        if len(X) > 10:
            lo = np.nanmin([min(X),min(Y),min(Z)])
            hi = np.nanmax([max(X),max(Y),max(Z)])
            if np.isfinite(lo) and np.isfinite(hi) and hi>lo:
                pad = 0.05*(hi-lo)
                ax1.set_ylim(lo-pad, hi+pad)

        # 更新平面轨迹
        scat.set_data(XYx, XYy)
        if len(XYx) > 10:
            xmin, xmax = np.nanmin(XYx), np.nanmax(XYx)
            ymin, ymax = np.nanmin(XYy), np.nanmax(XYy)
            if np.isfinite(xmin) and np.isfinite(xmax) and np.isfinite(ymin) and np.isfinite(ymax):
                xr = xmax-xmin; yr = ymax-ymin
                xr = 1.0 if xr==0 else xr
                yr = 1.0 if yr==0 else yr
                padx = 0.1*xr; pady = 0.1*yr
                ax2.set_xlim(xmin-padx, xmax+padx)
                ax2.set_ylim(ymin-pady, ymax+pady)

        return lineX, lineY, lineZ, scat

    ani = FuncAnimation(plt.gcf(), update, interval=20, blit=False)
    print("[INFO] 按 Ctrl+C 退出。保存文件：{}".format(args.save if args.save else "未开启"))
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if fout: fout.close()
        ser.close()

if __name__ == "__main__":
    main()
