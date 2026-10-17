"""
RGB-D 采集与对齐验证（第 1 周验收脚本）

跑法：
    E:\\lerobot-env\\Scripts\\python.exe E:\\lerobot-arm-test\\vision\\01_capture_rgbd.py

做什么：
  1. 开 640x480@30 的彩色 + 深度
  2. 把深度图对齐到彩色（rs.align），这样同一像素坐标下 color 和 depth 是同一个点
  3. 连续采 N 帧，统计实际帧率和丢帧
  4. 存 color.png / depth.png（uint16，单位 mm）/ intrinsics.json
  5. 存一张深度伪彩预览，方便肉眼检查

验收标准（plan.html 第 1 周）：
  - 连续 100 帧不丢帧
  - 深度图与彩色图像素对齐

注意：深度像素值 0 表示"无效"（黑、镜面、透明、太近太远），不是距离 0。
      做后续处理时一定要先把 0 滤掉，否则算出来的平均深度会偏小。
"""

import json
import os
import sys
import time

# Windows 控制台默认 GBK，中文会乱码
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import cv2
import numpy as np
import pyrealsense2 as rs

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")
WIDTH, HEIGHT, FPS = 640, 480, 30
N_FRAMES = 100


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)

    # 预热：D435i 的自动曝光需要几帧才稳定，前几帧不要用
    print("预热 30 帧 ...")
    for _ in range(30):
        pipe.wait_for_frames()

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"深度单位: {depth_scale * 1000:.4f} mm/unit")

    print(f"开始采集 {N_FRAMES} 帧 ...")
    t0 = time.time()
    frames_ok = 0
    first_color = None
    first_depth = None
    valid_ratios = []

    for _ in range(N_FRAMES):
        frames = pipe.wait_for_frames(timeout_ms=5000)
        aligned = align.process(frames)

        d = aligned.get_depth_frame()
        c = aligned.get_color_frame()
        if not d or not c:
            continue

        frames_ok += 1
        depth_img = np.asanyarray(d.get_data())
        color_img = np.asanyarray(c.get_data())

        if first_color is None:
            first_color = color_img.copy()
            first_depth = depth_img.copy()

        # 有效深度比例（0 = 无效）
        valid = np.count_nonzero(depth_img) / depth_img.size
        valid_ratios.append(valid)

    elapsed = time.time() - t0
    pipe.stop()

    fps_actual = frames_ok / elapsed
    print("\n" + "=" * 60)
    print(f"采集帧数     : {frames_ok} / {N_FRAMES}")
    print(f"耗时         : {elapsed:.2f} s")
    print(f"实际帧率     : {fps_actual:.1f} fps  (目标 {FPS})")
    print(f"有效深度像素 : {np.mean(valid_ratios) * 100:.1f}% "
          f"(最低 {np.min(valid_ratios) * 100:.1f}%)")

    if frames_ok < N_FRAMES:
        print(f"!! 丢帧 {N_FRAMES - frames_ok} 帧 —— 检查 USB 带宽或延长线")

    # ---- 保存 ----
    cv2.imwrite(os.path.join(OUT_DIR, "color.png"), first_color)
    cv2.imwrite(os.path.join(OUT_DIR, "depth.png"), first_depth)

    vis = cv2.applyColorMap(
        cv2.convertScaleAbs(first_depth, alpha=0.03), cv2.COLORMAP_JET
    )
    # 无效像素涂黑，避免被当成"最近的物体"
    vis[first_depth == 0] = (0, 0, 0)
    cv2.imwrite(os.path.join(OUT_DIR, "depth_vis.png"), vis)

    # 叠加图：彩色与深度伪彩各占一半。用来肉眼验收"像素对齐"——
    # 如果对齐错了，桌面边缘和物体轮廓会重影、错位。
    blend = cv2.addWeighted(first_color, 0.5, vis, 0.5, 0)
    cv2.imwrite(os.path.join(OUT_DIR, "overlay.png"), blend)

    # ---- 保存内参 ----
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    dintr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    color_to_depth = profile.get_stream(
        rs.stream.depth
    ).as_video_stream_profile().get_extrinsics_to(
        profile.get_stream(rs.stream.color).as_video_stream_profile()
    )

    meta = {
        "width": WIDTH,
        "height": HEIGHT,
        "fps": FPS,
        "depth_scale_m_per_unit": depth_scale,
        # pyrealsense2 用 ppx/ppy 表示主点，等价于 OpenCV 的 cx/cy
        "color_intrinsics": {
            "fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy,
            "model": str(intr.model),
            "coeffs": list(intr.coeffs),
        },
        "depth_intrinsics": {
            "fx": dintr.fx, "fy": dintr.fy, "cx": dintr.ppx, "cy": dintr.ppy,
            "model": str(dintr.model),
            "coeffs": list(dintr.coeffs),
        },
        # extrinsics.rotation 是长度 9 的扁平列表，按行重排成 3x3
        "depth_to_color_rotation": [
            list(color_to_depth.rotation[i * 3:(i + 1) * 3]) for i in range(3)
        ],
        "depth_to_color_translation_m": list(color_to_depth.translation),
        "note": "depth 已用 rs.align(rs.stream.color) 对齐到彩色，像素坐标一一对应",
    }
    with open(os.path.join(OUT_DIR, "intrinsics.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print(f"\n保存到: {OUT_DIR}")
    print("  color.png / depth.png / depth_vis.png / intrinsics.json")
    print(f"彩色内参 fx={intr.fx:.2f} fy={intr.fy:.2f} "
          f"cx={intr.ppx:.2f} cy={intr.ppy:.2f}")

    # ---- 平面度粗检：桌面应该是平的 ----
    # 这个数字直接告诉你 D435i 在 0.5 m 处的实际几何质量
    valid = first_depth[first_depth > 0]
    if valid.size > 0:
        print(f"\n深度范围: {valid.min()} ~ {valid.max()} mm, "
              f"中位数 {np.median(valid):.0f} mm")


if __name__ == "__main__":
    main()
