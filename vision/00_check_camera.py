"""
D435i 连通性检查（只读，不改变任何状态）

跑法：
    E:\\lerobot-env\\Scripts\\python.exe E:\\lerobot-arm-test\\vision\\00_check_camera.py

输出：设备列表、固件版本、以及所有可用流配置。
第 1 周第一项任务的验收依据。
"""

import pyrealsense2 as rs


def main():
    ctx = rs.context()
    devices = ctx.query_devices()

    print(f"pyrealsense2 版本: {rs.__version__}")
    print(f"发现设备数: {len(devices)}")
    print("=" * 70)

    if len(devices) == 0:
        print("没有找到 RealSense 设备。检查：")
        print("  1. USB 是否插在蓝色 USB 3.0 口上（不是前面板扩展口）")
        print("  2. 是否被别的程序占用（RealSense Viewer / 上位机）")
        return

    for i, dev in enumerate(devices):
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        fw = dev.get_info(rs.camera_info.firmware_version)
        usb = dev.get_info(rs.camera_info.usb_type_descriptor)
        print(f"[{i}] {name}")
        print(f"    序列号  : {serial}")
        print(f"    固件    : {fw}")
        print(f"    USB     : {usb}   <-- 必须是 3.x")

        sensors = dev.query_sensors()
        print(f"    传感器  : {len(sensors)} 个")
        for s in sensors:
            print(f"      - {s.get_info(rs.camera_info.name)}")

        depth_sensor = dev.first_depth_sensor()
        scale = depth_sensor.get_depth_scale()
        print(f"    深度单位: {scale * 1000:.3f} mm/unit")

        # 列出所有支持的流配置
        for stream_type, label in [
            (rs.stream.depth, "DEPTH"),
            (rs.stream.color, "COLOR"),
            (rs.stream.infrared, "INFRARED"),
        ]:
            profiles = []
            for p in dev.query_sensors():
                try:
                    for prof in p.get_stream_profiles():
                        if prof.stream_type() == stream_type:
                            v = prof.as_video_stream_profile()
                            profiles.append(
                                f"{v.width()}x{v.height()}@{prof.fps()}fps "
                                f"{prof.format()}"
                            )
                except Exception:
                    pass
            uniq = sorted(set(profiles))
            if uniq:
                print(f"    {label} 配置（{len(uniq)} 种）:")
                for u in uniq:
                    print(f"      {u}")
        print("-" * 70)


if __name__ == "__main__":
    main()
