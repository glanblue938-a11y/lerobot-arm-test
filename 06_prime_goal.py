#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使能扭矩前的安全检查：把 Goal_Position 写成当前位置

=========================================================================
【它解决什么问题】

舵机在 Torque_Enable 0 -> 1 的瞬间，会**驱动到 Goal_Position**。
如果 Goal_Position 和当前位置差得远，机械臂就会"蹿"过去。

实测（2026-10-17）：
  · 从臂 COM23：shoulder_lift 的 Goal 比当前位置差 **331 tick (29.1°)**，
                其余 5 个关节只差 ≤4 tick
  · 主臂 COM32：6 个关节的 Goal_Position **全是 0**，而当前位置是 906~3074
                —— 一旦使能扭矩就是 270° 的甩动

而 `SO101Follower.connect()` 跑 `configure()` 时会走
`with self.bus.torque_disabled(): ...`，这个上下文管理器在 **finally 里会把扭矩
重新打开**（lerobot/motors/motors_bus.py:677-691）。所以从臂接连完就是使能状态，
那 29° 会真的蹿出来。

【做法】

把每个关节的 Goal_Position(地址 42) 写成它自己的 Present_Position(地址 56)。
因为写完之后 Goal == Present，之后无论扭矩什么时候开，臂都**停在原地**。

【注意：写 Goal 会自动开扭矩】

实测（2026-10-17，主臂 ID 1）：扭矩为 0 时写一次 Goal_Position，
Torque_Enable 会自己从 0 变成 1。官方文档没写这一点。

所以本脚本 --yes 执行完之后，臂是**硬的**，不是松的。
安全仍然成立——Goal 当场等于 Present，使能瞬间不会动——但别再以为
"扭矩还是关的"。只要不打算马上用，就不要随便写 Goal。

这也是 lerobot 自己没做、但 `02_joint_test.py` 一直遵守的顺序：
    读当前位置 -> 目标位置 := 当前位置 -> 再使能扭矩

【用法】

  # 只看差值，不写任何东西
  python 06_prime_goal.py --port COM23

  # 真正写入（扭矩必须已关闭）
  python 06_prime_goal.py --port COM23 --yes

  # 关闭所有关节的扭矩（臂变松，可以用手扳）
  python 06_prime_goal.py --port COM23 --release --yes

【安全】

  · 默认只读。加 --yes 才写。
  · 写入前逐个确认 Torque_Enable == 0，有一个开着就**整体拒绝执行**
    （扭矩开着的时候写目标位置是有风险的，先断电重上电再说）。
  · 不主动写 Torque_Enable，不碰 EEPROM，不碰校准。
    —— 但要知道：写 Goal_Position 会让舵机固件自己把扭矩打开（见上）。
=========================================================================
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：  pip install pyserial")

for _s in (sys.stdout, sys.stderr):
    try:
        if _s.isatty():
            _s.reconfigure(errors="replace")
        else:
            _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

ADDR_GOAL_POSITION = 42
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56

JOINT_NAMES = {
    1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
    4: "wrist_flex", 5: "wrist_roll", 6: "gripper",
}

DEG_PER_TICK = 360.0 / 4096


def checksum(payload: bytes) -> int:
    return (~sum(payload)) & 0xFF


def build_packet(motor_id: int, instruction: int, params: bytes = b"") -> bytes:
    body = bytes([motor_id, len(params) + 2, instruction]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])


def read_status(ser, timeout: float = 0.05):
    deadline = time.time() + timeout
    seen = 0
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\xff":
            seen += 1
            if seen >= 2:
                break
        else:
            seen = 0
    else:
        return None
    rest = ser.read(2)
    if len(rest) < 2:
        return None
    motor_id, length = rest[0], rest[1]
    if length < 2:
        return None
    payload = ser.read(length)
    if len(payload) < length:
        return None
    body, chk = payload[:-1], payload[-1]
    if checksum(bytes([motor_id, length]) + body) != chk:
        return None
    return {"id": motor_id, "error": body[0], "params": body[1:]}


def read_reg(ser, motor_id: int, address: int, length: int):
    ser.reset_input_buffer()
    ser.write(build_packet(motor_id, INST_READ, bytes([address, length])))
    pkt = read_status(ser)
    if pkt is None or pkt["id"] != motor_id or len(pkt["params"]) < length:
        return None
    return int.from_bytes(pkt["params"][:length], "little", signed=False)


def write_reg(ser, motor_id: int, address: int, value: int, length: int = 2) -> bool:
    params = bytes([address]) + int(value).to_bytes(length, "little", signed=False)
    ser.reset_input_buffer()
    ser.write(build_packet(motor_id, INST_WRITE, params))
    pkt = read_status(ser)
    return pkt is not None and pkt["id"] == motor_id


def release_torque(args) -> int:
    """关闭所有关节的扭矩（地址 40 = 0）。

    【什么时候需要】
    lerobot 正常退出（Ctrl+C）会走 disconnect() 关扭矩。但如果进程是**被强杀**的
    （任务管理器结束进程、后台任务被 kill），disconnect() 根本没跑，
    关节就会一直保持使能 —— 臂是硬的、持续通电发热，还在跟重力较劲。

    实测：2026-10-17 主臂从臂遥操作会话被强杀后，从臂 6 个关节扭矩全是 1。

    【安全】关扭矩会让臂立刻失去支撑，可能直接落下来。确认下面没东西、
    或者用手托住，再执行。
    """
    print("=" * 92)
    print(f"  关闭扭矩  ·  {args.port}")
    print("=" * 92)

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        time.sleep(0.2)
        before = {i: read_reg(ser, i, ADDR_TORQUE_ENABLE, 1) for i in range(1, 7)}

        print(f"\n  {'ID':<3} {'关节名':<15} {'当前扭矩':>9}")
        print("  " + "-" * 30)
        for i in range(1, 7):
            print(f"  {i:<3} {JOINT_NAMES[i]:<15} {str(before[i]):>9}")

        on = [i for i, v in before.items() if v == 1]
        if not on:
            print("\n  [OK] 所有关节扭矩本来就是关的，不用处理。")
            return 0

        if not args.yes:
            print(f"\n  [dry-run] 会把 {on} 这 {len(on)} 个关节的扭矩关掉。")
            print("           **关扭矩后臂会立刻失去支撑** —— 确认下面没东西或用手托住。")
            print("           确认后加 --yes 执行。")
            return 0

        print(f"\n  关闭中（{len(on)} 个关节）...")
        fail = []
        for i in on:
            if not write_reg(ser, i, ADDR_TORQUE_ENABLE, 0, length=1):
                fail.append(i)

        time.sleep(0.3)
        after = {i: read_reg(ser, i, ADDR_TORQUE_ENABLE, 1) for i in range(1, 7)}

        print(f"\n  {'ID':<3} {'关节名':<15} {'改前':>6} {'改后':>6} {'校验':>8}")
        print("  " + "-" * 40)
        bad = 0
        for i in range(1, 7):
            ok = after[i] == 0
            if not ok:
                bad += 1
            print(f"  {i:<3} {JOINT_NAMES[i]:<15} {str(before[i]):>6} "
                  f"{str(after[i]):>6} {'OK' if ok else '失败':>8}")

        if fail:
            print(f"\n  [!!] 没收到应答的关节：{fail}")
        print()
        if bad == 0 and not fail:
            print("  [OK] 扭矩全部关闭，臂现在可以用手自由扳动。")
            return 0
        print(f"  [!!] 有 {bad + len(fail)} 个关节没关掉。")
        return 1


def main() -> int:
    ap = argparse.ArgumentParser(description="使能扭矩前把 Goal_Position 对齐到当前位置")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--yes", action="store_true", help="真正写入，否则只预览")
    ap.add_argument("--release", action="store_true",
                    help="关闭所有关节的扭矩（lerobot 被强杀后用它收尾）")
    ap.add_argument("--threshold", type=int, default=20,
                    help="差值超过这么多 tick 才认为需要处理（默认 20）")
    args = ap.parse_args()

    assert build_packet(1, INST_PING) == b"\xff\xff\x01\x02\x01\xfb", "协议自检失败"

    if args.release:
        return release_torque(args)

    print("=" * 92)
    print(f"  Goal_Position 对齐检查  ·  {args.port}")
    print("=" * 92)

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        time.sleep(0.2)

        state = {}
        for i in range(1, 7):
            goal = read_reg(ser, i, ADDR_GOAL_POSITION, 2)
            pres = read_reg(ser, i, ADDR_PRESENT_POSITION, 2)
            te = read_reg(ser, i, ADDR_TORQUE_ENABLE, 1)
            state[i] = (goal, pres, te)

        if all(v[1] is None for v in state.values()):
            print("  [!!] 一个舵机都没应答，检查端口和供电。")
            return 1

        print(f"\n  {'ID':<3} {'关节名':<15} {'Goal':>7} {'Present':>8} {'差值':>8} "
              f"{'角度差':>9} {'扭矩':>5} {'判定':>10}")
        print("  " + "-" * 74)

        need, blocked, missing = [], [], []
        worst = 0
        for i in range(1, 7):
            goal, pres, te = state[i]
            if pres is None:
                missing.append(i)
                print(f"  {i:<3} {JOINT_NAMES[i]:<15} {'—':>7} {'—':>8} {'—':>8} "
                      f"{'—':>9} {'—':>5} {'无应答':>10}")
                continue
            if goal is None:
                missing.append(i)
                print(f"  {i:<3} {JOINT_NAMES[i]:<15} {'读不到':>7} {pres:>8} "
                      f"{'?':>8} {'?':>9} {str(te):>5} {'Goal不可读':>10}")
                continue
            delta = goal - pres
            worst = max(worst, abs(delta))
            if te == 1:
                blocked.append(i)
                verdict = "扭矩开着"
            elif abs(delta) > args.threshold:
                need.append(i)
                verdict = "需对齐"
            else:
                verdict = "已对齐"
            print(f"  {i:<3} {JOINT_NAMES[i]:<15} {goal:>7} {pres:>8} {delta:>8} "
                  f"{delta * DEG_PER_TICK:>8.1f}° {str(te):>5} {verdict:>10}")

        print()

        if blocked:
            print(f"  [!!] 这些关节的扭矩是开着的：{blocked}")
            print("       扭矩开着的时候不写目标位置。先断电重上电（上电后应保持松开状态），")
            print("       再跑本脚本。")
            return 1

        if missing:
            print(f"  [!]  这些关节读不到 Goal/Present：{missing}")

        if not need:
            print(f"  [OK] 所有读到的关节差值都 <= {args.threshold} tick，"
                  f"使能扭矩时不会蹿。不需要处理。")
            return 0

        print(f"  需要对齐的关节：{need}")
        print(f"  最大差值 {worst} tick = {worst * DEG_PER_TICK:.1f}°")

        if not args.yes:
            print("\n  [dry-run] 没有写任何东西。确认后加 --yes 执行。")
            return 0

        print("\n  写入中（Goal_Position := Present_Position）...")
        ok, fail = [], []
        for i in need:
            _, pres, _ = state[i]
            if write_reg(ser, i, ADDR_GOAL_POSITION, pres):
                ok.append(i)
            else:
                fail.append(i)

        time.sleep(0.3)
        print(f"\n  {'ID':<3} {'关节名':<15} {'Goal':>7} {'Present':>8} {'差值':>8} {'校验':>8}")
        print("  " + "-" * 58)
        bad = 0
        for i in range(1, 7):
            goal = read_reg(ser, i, ADDR_GOAL_POSITION, 2)
            pres = read_reg(ser, i, ADDR_PRESENT_POSITION, 2)
            if goal is None or pres is None:
                print(f"  {i:<3} {JOINT_NAMES[i]:<15} {'—':>7} {'—':>8} {'—':>8} {'读不到':>8}")
                bad += 1
                continue
            d = goal - pres
            good = abs(d) <= args.threshold
            if not good:
                bad += 1
            print(f"  {i:<3} {JOINT_NAMES[i]:<15} {goal:>7} {pres:>8} {d:>8} "
                  f"{'OK' if good else '仍差':>8}")

        if fail:
            print(f"\n  [!!] 写入没收到应答的关节：{fail}")
        print()
        if bad == 0 and not fail:
            print("  [OK] 全部对齐。现在使能扭矩不会蹿了。")
            print("       可以安全地跑 lerobot-calibrate / lerobot-teleoperate / lerobot-record。")
            return 0
        print(f"  [!]  有 {bad} 个关节还没对齐，看看上面的表。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
