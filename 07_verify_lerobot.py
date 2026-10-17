#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
整机验收：用 lerobot 自己的 Robot / Teleoperator 类连一遍

=========================================================================
【它验证什么】

前面 01~06 都是裸串口协议，验证的是"硬件和舵机配置"。这一步换成
**lerobot 真正会用的那几个类**，验证的是"lerobot 能不能用这台臂"：

  · SO101Follower / SO101Leader 能不能 connect()
    （这正是之前失败的地方：主臂电压错误 -> 握手把舵机全过滤掉 -> 发现 0 个）
  · is_calibrated 是不是 True
    （判断依据是「calibration 文件里的值 == 舵机 EEPROM 里的值」，
      所以 --install 装上前任的校准后应该直接为真）
  · get_observation() 能不能读出归一化后的关节值
    （说明校准值是自洽的、能算得出来，不会出现除零或超范围）

【注意】这一步会**写**东西 —— 但写的是 lerobot 每次运行都会写的运动参数，
      不是校准值：
        connect() -> configure() -> Return_Delay_Time / Acceleration /
        Maximum_Acceleration / Phase / Operating_Mode / P-I-D 增益
        (+ 从臂夹爪的 Max_Torque_Limit / Protection_Current / Overload_Torque)
      **Homing_Offset / Min_Position_Limit / Max_Position_Limit 一个都不碰。**

      从臂连完后扭矩会被**打开**（lerobot 的 torque_disabled 上下文在 finally
      里重新使能），臂会变硬并保持当前姿态。跑之前请先执行 06_prime_goal.py
      对齐 Goal_Position，否则会蹿。

【用法】
  python 07_verify_lerobot.py --follower-port COM23 --leader-port COM32
  python 07_verify_lerobot.py --follower-port COM23          # 只验从臂
  python 07_verify_lerobot.py --leader-port COM32            # 只验主臂
=========================================================================
"""

from __future__ import annotations

import argparse
import sys
import traceback

for _s in (sys.stdout, sys.stderr):
    try:
        if _s.isatty():
            _s.reconfigure(errors="replace")
        else:
            _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.teleoperators.so_leader.config_so_leader import SO101LeaderConfig
from lerobot.teleoperators.so_leader.so_leader import SO101Leader


def check_follower(port: str, arm_id: str) -> bool:
    print("=" * 92)
    print(f"  从臂 follower  ·  {port}  ·  id={arm_id}")
    print("=" * 92)
    cfg = SO101FollowerConfig(port=port, id=arm_id)
    robot = SO101Follower(cfg)
    ok = True
    try:
        robot.connect()
    except Exception:
        print("  [!!] connect() 失败：")
        traceback.print_exc()
        return False
    try:
        print(f"  connect()          -> OK")
        print(f"  is_calibrated      -> {robot.is_calibrated}")

        if not robot.is_calibrated:
            ok = False
            print("       [!!] 不是 True —— calibration 文件和舵机 EEPROM 对不上。")

        obs = robot.get_observation()
        keys = sorted(k for k in obs if k.endswith(".pos"))
        print(f"  get_observation()  -> {len(keys)} 个关节")
        print(f"\n  {'关节':<20} {'归一化值':>12}")
        print("  " + "-" * 34)
        for k in keys:
            v = obs[k]
            print(f"  {k:<20} {v:>12.3f}")
        print(f"\n  {'[OK]' if ok else '[!!]'} 从臂验收{'通过' if ok else '未通过'}")
    except Exception:
        print("  [!!] 读取过程出错：")
        traceback.print_exc()
        ok = False
    finally:
        try:
            robot.disconnect()
            print("  已断开（扭矩已关闭，臂现在可以用手自由扳动）")
        except Exception as e:
            print(f"  断开时出错：{e}")
    print()
    return ok


def check_leader(port: str, arm_id: str) -> bool:
    print("=" * 92)
    print(f"  主臂 leader  ·  {port}  ·  id={arm_id}")
    print("=" * 92)
    cfg = SO101LeaderConfig(port=port, id=arm_id)
    tele = SO101Leader(cfg)
    ok = True
    try:
        tele.connect()
    except Exception:
        print("  [!!] connect() 失败：")
        traceback.print_exc()
        return False
    try:
        print(f"  connect()          -> OK")
        print(f"  is_calibrated      -> {tele.is_calibrated}")
        if not tele.is_calibrated:
            ok = False
            print("       [!!] 不是 True —— calibration 文件和舵机 EEPROM 对不上。")

        act = tele.get_action()
        keys = sorted(k for k in act if k.endswith(".pos"))
        print(f"  get_action()       -> {len(keys)} 个关节")
        print(f"\n  {'关节':<20} {'归一化值':>12}")
        print("  " + "-" * 34)
        for k in keys:
            print(f"  {k:<20} {act[k]:>12.3f}")
        print(f"\n  {'[OK]' if ok else '[!!]'} 主臂验收{'通过' if ok else '未通过'}")
    except Exception:
        print("  [!!] 读取过程出错：")
        traceback.print_exc()
        ok = False
    finally:
        try:
            tele.disconnect()
            print("  已断开（主臂本来就是自由状态）")
        except Exception as e:
            print(f"  断开时出错：{e}")
    print()
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="用 lerobot 的类做整机验收")
    ap.add_argument("--follower-port", help="从臂串口，如 COM23")
    ap.add_argument("--follower-id", default="follower1")
    ap.add_argument("--leader-port", help="主臂串口，如 COM32")
    ap.add_argument("--leader-id", default="leader1")
    args = ap.parse_args()

    if not args.follower_port and not args.leader_port:
        ap.error("至少给一个 --follower-port 或 --leader-port")

    results = {}
    if args.follower_port:
        results["从臂"] = check_follower(args.follower_port, args.follower_id)
    if args.leader_port:
        results["主臂"] = check_leader(args.leader_port, args.leader_id)

    print("=" * 92)
    print("  结论")
    print("=" * 92)
    for k, v in results.items():
        print(f"  {k}: {'[OK] 通过' if v else '[!!] 未通过'}")
    if all(results.values()):
        print("\n  lerobot 可以正常使用这两台臂了。")
        print("  下一步：")
        print("    E:/lerobot-env/Scripts/lerobot-teleoperate \\")
        print(f"      --robot.type=so101_follower --robot.port={args.follower_port} "
              f"--robot.id={args.follower_id} \\")
        print(f"      --teleop.type=so101_leader --teleop.port={args.leader_port} "
              f"--teleop.id={args.leader_id}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
