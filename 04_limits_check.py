#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
限位 / 扭矩诊断（只读）

=========================================================================
【安全保证】只发送 PING(0x01) 和 READ(0x02)，**从不发送 WRITE(0x03)**。
机械臂不会有任何运动。
=========================================================================

【它回答什么】
  02_joint_test.py 报"未能到达目标位置"时，有两种完全不同的可能：

    A. 舵机坏了 / 扭矩不足 / 机械卡涩   -> 要修
    B. 被 EEPROM 里的软限位挡住了        -> 完全正常，是配置问题

  这两者在故障现象上一模一样（都是"不动"），但处理方式天差地别。
  单看 02 的输出无法区分，必须去读 EEPROM 里的限位值。

【要看的寄存器】（地址来自 lerobot/motors/feetech/tables.py）
  Min_Position_Limit  (9,  2)  EEPROM  出厂/校准写入的最小软限位
  Max_Position_Limit  (11, 2)  EEPROM  最大软限位
  Max_Torque_Limit    (16, 2)  EEPROM  扭矩上限（0~1000，1000 = 100%）
  Torque_Limit        (48, 2)  SRAM    运行时扭矩限幅
  Homing_Offset       (31, 2)  EEPROM  中位偏移，影响位置零点
  Present_Position    (56, 2)  当前位置
  Present_Load        (60, 2)  当前负载
  Present_Velocity    (58, 2)  当前速度（判断是不是还在动）

【关键判据】
  Min_Position_Limit <= Present_Position <= Max_Position_Limit  ?
  如果当前位置正好顶在某一端，那么朝那个方向的目标值会被舵机直接忽略 ——
  表现为"一个 tick 都不动"，这是正常保护行为，不是故障。

【用法】
  python 04_limits_check.py --port COM23
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
# INST_WRITE = 0x03   # <- 故意不实现

JOINT_NAMES = {
    1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
    4: "wrist_flex", 5: "wrist_roll", 6: "gripper",
}

ADDR = {
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Torque_Limit": (16, 2),
    "Homing_Offset": (31, 2),
    "Torque_Limit": (48, 2),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Current": (69, 2),
    "Moving": (66, 1),
}

# 运动/保护相关参数。关节"走到一半停住"时，答案通常在这一组里：
#   CW/CCW_Dead_Zone      死区。舵机认为"已经到位"的容差，单位 tick。
#                         **死区偏大是"差十几个 tick 就停"的头号嫌疑**，
#                         因为舵机内部认为到位了就不再出力。
#   Minimum_Startup_Force 启动最小力。太小则起步困难，太大力会抖。
#   Protection_Current    过流保护阈值，超了会切扭矩。
#   Protective_Torque     保护扭矩百分比。
#   Overload_Torque       过载扭矩百分比。
#   Operating_Mode        0=位置伺服，3=步进，2=PWM。位置模式才该是 0/1。
ADDR_MOTION = {
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),
    "Operating_Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Overload_Torque": (36, 1),
    "Acceleration": (41, 1),
    "Maximum_Velocity_Limit": (84, 1),
}

TICKS_PER_TURN = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_TURN

# 距限位这么近就算"贴住了"。不能用严格判据：
# 实测 ID 3 停在 3185、上限 3188，只差 3 tick，严格判据会判成"正常"，
# 但舵机已经吃不下任何测试幅度了。
LIMIT_MARGIN = 10


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


def to_signed16(raw: int) -> int:
    return raw - 65536 if raw >= 32768 else raw


def decode_load(raw: int | None) -> int | None:
    if raw is None:
        return None
    magnitude = raw & 0x3FF
    return -magnitude if raw & 0x400 else magnitude


def main() -> int:
    ap = argparse.ArgumentParser(description="限位/扭矩诊断（只读，不会让机械臂运动）")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    args = ap.parse_args()

    assert build_packet(1, INST_PING) == b"\xff\xff\x01\x02\x01\xfb", "协议自检失败"

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        time.sleep(0.2)
        ids = [i for i in range(1, 7)]
        found = [i for i in ids if read_reg(ser, i, *ADDR["Present_Position"]) is not None]
        if not found:
            print("[!!] 没有发现舵机")
            return 1

        v = {}
        for i in found:
            v[i] = {k: read_reg(ser, i, *a) for k, a in ADDR.items()}

        print("=" * 96)
        print("  软限位 / 扭矩配置")
        print("=" * 96)
        print(f"  {'ID':<3} {'关节名':<15} {'最小限位':>9} {'最大限位':>9} "
              f"{'当前位置':>9} {'扭矩上限':>9} {'扭矩限幅':>9} {'中位偏移':>9}")
        print("  " + "-" * 92)

        at_limit = {}
        for i in found:
            d = v[i]
            name = JOINT_NAMES.get(i, "?")
            mn, mx = d["Min_Position_Limit"], d["Max_Position_Limit"]
            pos = d["Present_Position"]
            mtl, tl = d["Max_Torque_Limit"], d["Torque_Limit"]
            ho = d["Homing_Offset"]

            def f(x, fn=str):
                return "  ?  " if x is None else str(fn(x))

            print(f"  {i:<3} {name:<15} "
                  f"{f(mn):>9} {f(mx):>9} {f(pos):>9} "
                  f"{f(mtl):>9} {f(tl):>9} {f(ho, to_signed16):>9}")

            if mn is not None and mx is not None and pos is not None:
                # 不能只在"正好等于/越过"限位时才报。
                # 实测 ID 3 停在 3185、上限 3188 —— 差 3 tick，严格判据会漏掉，
                # 但舵机已经吃不下一个正常的测试幅度了，必须按"贴住限位"处理。
                if pos <= mn + LIMIT_MARGIN:
                    at_limit[i] = "下限"
                elif pos >= mx - LIMIT_MARGIN:
                    at_limit[i] = "上限"

        print()
        print("=" * 96)
        print("  逐个关节判读")
        print("=" * 96)
        for i in found:
            d = v[i]
            name = JOINT_NAMES.get(i, "?")
            mn, mx, pos = d["Min_Position_Limit"], d["Max_Position_Limit"], d["Present_Position"]
            load = decode_load(d["Present_Load"])
            vel = d["Present_Velocity"]
            vel = to_signed16(vel) if vel is not None else None
            moving = d["Moving"]
            ho = d["Homing_Offset"]
            ho = to_signed16(ho) if ho is not None else None

            print()
            print(f"  ID {i}  {name}")
            if mn is None or mx is None or pos is None:
                print("      [!!] 关键寄存器读不到")
                continue
            span = mx - mn
            print(f"      限位区间  {mn} ~ {mx}   (跨度 {span} tick = {span * DEG_PER_TICK:.1f}°)")
            print(f"      当前位置  {pos}  ({pos * DEG_PER_TICK:.1f}°)")
            print(f"      距下限    {pos - mn} tick     距上限    {mx - pos} tick")
            print(f"      负载 {load}   速度 {vel}   移动标志 {moving}   中位偏移 {ho}")

            if i in at_limit:
                end = at_limit[i]
                print(f"      [!!] 当前位置**顶在{end}**上。朝这个方向发目标值会被舵机直接忽略，")
                print(f"           表现就是 02_joint_test.py 里的「一个 tick 都不动」。")
                print(f"           这是软限位保护，**不是故障**。")
            elif span < TICKS_PER_TURN * 0.5:
                print(f"      [!] 限位跨度偏小（{span * DEG_PER_TICK:.1f}°），行程被限制了。")
            else:
                print(f"      [OK] 不在限位上，两侧都有余量。")

        print()
        print("=" * 96)
        print("  运动 / 保护参数")
        print("=" * 96)
        mo = {}
        for i in found:
            mo[i] = {k: read_reg(ser, i, *a) for k, a in ADDR_MOTION.items()}

        print(f"  {'ID':<3} {'关节名':<15} {'死区CW':>7} {'死区CCW':>8} {'启动力':>7} "
              f"{'过流保护':>9} {'保护扭矩':>9} {'过载扭矩':>9} {'工作模式':>9} {'加速':>6}")
        print("  " + "-" * 100)
        for i in found:
            d = mo[i]
            name = JOINT_NAMES.get(i, "?")

            def f(x):
                return "  ?  " if x is None else str(x)

            print(f"  {i:<3} {name:<15} "
                  f"{f(d['CW_Dead_Zone']):>7} {f(d['CCW_Dead_Zone']):>8} "
                  f"{f(d['Minimum_Startup_Force']):>7} {f(d['Protection_Current']):>9} "
                  f"{f(d['Protective_Torque']):>9} {f(d['Overload_Torque']):>9} "
                  f"{f(d['Operating_Mode']):>9} {f(d['Acceleration']):>6}")

        # 死区是"差十几个 tick 就停"的头号嫌疑，单独标出来
        big_dead = [(i, max(mo[i]["CW_Dead_Zone"] or 0, mo[i]["CCW_Dead_Zone"] or 0))
                    for i in found]
        big_dead = [(i, v) for i, v in big_dead if v >= 10]
        if big_dead:
            print()
            for i, v in big_dead:
                print(f"  [!] ID {i} ({JOINT_NAMES.get(i, '?')}) 死区 = {v} tick "
                      f"({v * DEG_PER_TICK:.2f}°)")
            print("      舵机认为「距目标小于死区」就算到位，于是不再出力 ——")
            print("      表现就是差十几个 tick 就停住。这是配置问题，不是故障。")

        print()
        print("=" * 96)
        print("  结论")
        print("=" * 96)
        if at_limit:
            for i, end in at_limit.items():
                d = v[i]
                pos = d["Present_Position"]
                edge = d["Min_Position_Limit"] if end == "下限" else d["Max_Position_Limit"]
                away = "正向（值变大）" if end == "下限" else "反向（值变小）"
                print(f"  ID {i} ({JOINT_NAMES.get(i, '?')}) 贴住{end}"
                      f"（{pos} vs {edge}，差 {abs(pos - edge)} tick）")
                print(f"      -> 朝{'下限' if end == '下限' else '上限'}方向没有行程余量；"
                      f"朝{away}仍有行程，能正常测")
            print()
            print("  说明：贴住限位本身**不是故障**，是软限位保护。")
            print("        02_joint_test.py 已改为自动避开顶死的方向，直接重跑即可。")
            print()
            print("  如果想真正放开行程（会覆盖前任使用者的校准），两选一：")
            print("    1) 重跑 lerobot 校准流程（lerobot-calibrate），它会重写限位")
            print("    2) 手工改：EEPROM 限位需先写 Lock(55)=0，改完记得写回 Lock=1")
            print("       改之前先读出来存好，否则丢了原始校准值就回不去了")
        else:
            print("  [OK] 没有关节顶在软限位上。")
            print("       那么 02 里的'不动'就得按真故障排查：扭矩上限、机械卡涩、供电。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
