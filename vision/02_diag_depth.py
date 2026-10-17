"""
深度质量诊断（只读）

对比三件事：
  1. 对齐前 vs 对齐后的有效像素比例
  2. 有效深度的分布（判断是"远到超量程"还是"根本没测到"）
  3. 有效像素在画面上的空间分布（看是不是大片区域失效）

跑法：
    E:\\lerobot-env\\Scripts\\python.exe E:\\lerobot-arm-test\\vision\\02_diag_depth.py
"""

import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 640, 480, 30
N = 20


def stats(depth, label):
    total = depth.size
    zero = np.count_nonzero(depth == 0)
    sat = np.count_nonzero(depth == 65535)
    valid = depth[(depth > 0) & (depth < 65535)]
    print(f"\n[{label}]")
    print(f"  0 (无效)     : {zero / total * 100:5.1f}%")
    print(f"  65535 (饱和) : {sat / total * 100:5.1f}%")
    print(f"  有效         : {valid.size / total * 100:5.1f}%")
    if valid.size:
        p = np.percentile(valid, [1, 5, 25, 50, 75, 95, 99])
        qs = [1, 5, 25, 50, 75, 95, 99]
        print("  有效深度 mm: " +
              " ".join(f"P{q}={v:.0f}" for q, v in zip(qs, p)))

    # 3x3 分块看空间分布：整块 0% 说明那片区域根本没测到
    h, w = depth.shape
    grid = []
    for r in range(3):
        row = []
        for c in range(3):
            blk = depth[r * h // 3:(r + 1) * h // 3,
                        c * w // 3:(c + 1) * w // 3]
            row.append(np.count_nonzero(blk) / blk.size * 100)
        grid.append(row)
    print("  空间有效分布 (3x3, %):")
    for row in grid:
        print("    " + "  ".join(f"{v:5.1f}" for v in row))


def main():
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)

    print("预热 30 帧 ...")
    for _ in range(30):
        pipe.wait_for_frames()

    ds = profile.get_device().first_depth_sensor()
    print(f"深度单位: {ds.get_depth_scale() * 1000:.4f} mm/unit")

    raw_stack, al_stack = [], []
    for _ in range(N):
        frames = pipe.wait_for_frames(timeout_ms=5000)
        aligned = align.process(frames)
        # 【关键】必须 copy()。np.asanyarray 对 get_data() 是零拷贝视图，
        # 持有它就等于持有整个 frame 对象；攒满帧池（D435i 是 16）之后
        # align.process() 会报 "Error occured during execution of the processing block"。
        raw_stack.append(np.array(frames.get_depth_frame().get_data()))
        al_stack.append(np.array(aligned.get_depth_frame().get_data()))

    pipe.stop()

    raw_med = np.median(np.stack(raw_stack), axis=0).astype(np.uint16)
    al_med = np.median(np.stack(al_stack), axis=0).astype(np.uint16)

    stats(raw_med, "对齐前 (原始深度)")
    stats(al_med, "对齐后 (depth->color)")


if __name__ == "__main__":
    main()
