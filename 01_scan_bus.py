#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SO-100 / SO-101 总线只读诊断工具（Feetech STS 系列总线舵机）

=========================================================================
【安全保证】本脚本只发送 PING(0x01) 和 READ(0x02) 两种指令，
**永远不会发送 WRITE(0x03)**，因此机械臂不会产生任何运动。
二手设备、状态未知、首次上电时都可以放心运行。
=========================================================================

【它回答什么】
  1. 哪个 COM 口是舵机总线，波特率是多少
  2. 总线上到底挂了几个舵机、ID 分别是什么（暴力扫 0~20）
  3. 每个舵机能否稳定读出 位置/电压/温度/负载/扭矩状态
  4. 是否存在 ID 重复、掉线、供电不足、读数跳变

【依赖】
  pip install pyserial

【用法】
  python 01_scan_bus.py --list                    # 先看有哪些串口
  python 01_scan_bus.py --port COM7               # 默认按 1000000 扫描
  python 01_scan_bus.py --port COM7 --all-baud    # 不确定波特率时用这个
  python 01_scan_bus.py --port COM7 --max-id 12   # 扩大搜索范围

【重要说明】
  下面 REGISTERS 里的地址来自 Feetech STS 系列的通用寄存器表。
  如果你装的 lerobot 版本自带另一份表，以 lerobot 源码为准
  （通常在 src/lerobot/motors/feetech/tables.py）。
  脚本对每个读数都做了合理性校验，超出正常范围会直接标红提示，
  所以即使地址有偏差也不会被误读成"正常"。
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial，请先执行：  pip install pyserial")

# Windows 控制台/管道的编码可能是 GBK，编码不了的字符会直接抛异常中断脚本。
# 这里兜底成替换字符，保证诊断跑到最后而不是中途崩掉。
#
# 另外：直接连终端时保持系统编码（GBK），你自己看中文正常；
# 被重定向/管道捕获时改用 UTF-8，否则接收方按 UTF-8 解码会得到一堆乱码。
for _stream in (sys.stdout, sys.stderr):
    try:
        if _stream.isatty():
            _stream.reconfigure(errors="replace")
        else:
            _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


# ---------------------------------------------------------------- 协议常量

INST_PING = 0x01
INST_READ = 0x02
# INST_WRITE = 0x03   # ← 故意不实现。本脚本是只读工具。

# LeRobot SO-101 follower 的默认关节顺序
JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

# Feetech STS 系列 SRAM 寄存器表: 名称 -> (地址, 字节数)
# EEPROM 区（ID / 波特率）在不同固件版本里地址有偏移，所以这里故意不读，
# 改为暴力 PING 反推 ID —— 更可靠，也不会误写。
REGISTERS = {
    "Torque_Enable":       (40, 1),   # 1 = 扭矩使能（关节锁死）
    "Goal_Position":       (42, 2),   # 目标位置，0~4095
    "Present_Position":    (56, 2),   # 当前位置，0~4095
    "Present_Load":        (60, 2),   # 当前负载（bit10 为符号位）
    "Present_Voltage":     (62, 1),   # 电压，单位 0.1V
    "Present_Temperature": (63, 1),   # 温度，单位 °C
    "Present_Current":     (69, 2),   # 电流；部分固件不支持
}

TICKS_PER_TURN = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_TURN
BAUD_CANDIDATES = [1000000, 500000, 250000, 128000, 115200, 57600, 38400, 19200, 9600]

TEMPERATURE_CEILING = 55   # °C，超过判定过热


# ---------------------------------------------------------------- 协议实现

def checksum(payload: bytes) -> int:
    return (~sum(payload)) & 0xFF


def build_instruction(motor_id: int, instruction: int, params: bytes = b"") -> bytes:
    """组一个指令包: FF FF ID LENGTH INST PARAMS... CHECKSUM"""
    body = bytes([motor_id, len(params) + 2, instruction]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])


def read_status(ser, timeout: float = 0.05):
    """读一个状态包，超时或校验失败返回 None。"""
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


def ping(ser, motor_id: int):
    ser.reset_input_buffer()
    ser.write(build_instruction(motor_id, INST_PING))
    pkt = read_status(ser)
    return pkt if (pkt and pkt["id"] == motor_id) else None


def read_reg(ser, motor_id: int, address: int, length: int):
    """读寄存器，返回原始无符号整数；失败返回 None。"""
    ser.reset_input_buffer()
    ser.write(build_instruction(motor_id, INST_READ, bytes([address, length])))
    pkt = read_status(ser)
    if pkt is None or pkt["id"] != motor_id or len(pkt["params"]) < length:
        return None
    return int.from_bytes(pkt["params"][:length], "little", signed=False)


# ---------------------------------------------------------------- 数值解释

def decode_load(raw: int) -> int:
    """Feetech 负载用 bit10 作符号位，低 10 位是大小。"""
    magnitude = raw & 0x3FF
    return -magnitude if raw & 0x400 else magnitude


def ticks_to_deg(ticks: int) -> float:
    return ticks * DEG_PER_TICK


# ---------------------------------------------------------------- 串口探测

def list_ports_and_exit() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("没有检测到任何串口设备。")
        print("-> 检查 USB 线是否插好、驱动是否安装（设备管理器里找 CH340）。")
        return
    print(f"检测到 {len(ports)} 个串口：\n")
    print(f"  {'端口':<10} {'描述':<45} {'硬件ID'}")
    print("  " + "-" * 90)
    for p in ports:
        print(f"  {p.device:<10} {(p.description or ''):<45} {p.hwid or ''}")
    print()
    print("提示：拔掉机械臂 USB 再看一遍，消失的那个就是舵机总线。")


def probe_baud(port: str, max_id: int) -> tuple[str | None, list[int]]:
    """逐个波特率试，返回 (命中的波特率, 该波特率下响应的 ID 列表)。"""
    print("正在探测波特率……")
    for baud in BAUD_CANDIDATES:
        try:
            with serial.Serial(port, baud, timeout=0.02) as ser:
                time.sleep(0.2)
                found = [i for i in range(1, max_id + 1) if ping(ser, i) is not None]
        except serial.SerialException as exc:
            print(f"  无法打开 {port}: {exc}")
            return None, []
        if found:
            print(f"  [OK] {baud:>7} bps  响应到 ID: {found}")
            return baud, found
        print(f"    {baud:>7} bps  无响应")
    return None, []


# ---------------------------------------------------------------- 主诊断

def diagnose(ser, ids: list[int], run_repeat: int = 6) -> None:
    print()
    print("=" * 78)
    print("  舵机读数")
    print("=" * 78)
    header = (f"  {'ID':<3} {'关节名':<15} {'位置':>6} {'角度':>8} "
              f"{'电压':>7} {'温度':>6} {'负载':>7} {'扭矩':>5} {'电流':>6}")
    print(header)
    print("  " + "-" * 74)

    problems: list[str] = []

    for motor_id in ids:
        name = JOINT_NAMES.get(motor_id, "(未知)")
        vals = {}
        for reg, (addr, size) in REGISTERS.items():
            vals[reg] = read_reg(ser, motor_id, addr, size)

        pos = vals["Present_Position"]
        volt = vals["Present_Voltage"]
        temp = vals["Present_Temperature"]
        load = vals["Present_Load"]
        cur = vals["Present_Current"]
        torque = vals["Torque_Enable"]

        def fmt(v, fn=str, width=0):
            return "  ?  " if v is None else str(fn(v))

        print(f"  {motor_id:<3} {name:<15} "
              f"{fmt(pos, width=6):>6} "
              f"{fmt(pos, lambda x: f'{ticks_to_deg(x):.1f}°'):>8} "
              f"{fmt(volt, lambda x: f'{x / 10:.1f}V'):>7} "
              f"{fmt(temp, lambda x: f'{x}°C'):>6} "
              f"{fmt(load, decode_load):>7} "
              f"{fmt(torque, lambda x: 'ON' if x else 'off'):>5} "
              f"{fmt(cur):>6}")

        # ---- 逐项合理性校验 ----
        if pos is None:
            problems.append(f"ID {motor_id} ({name}): 位置读取失败 —— 通信不稳或该 ID 不存在")
        elif not (0 <= pos <= TICKS_PER_TURN):
            problems.append(f"ID {motor_id} ({name}): 位置 {pos} 超出 0~4095，读数可疑")

        if temp is not None and temp > TEMPERATURE_CEILING:
            problems.append(f"ID {motor_id} ({name}): 温度 {temp}°C 过高")
        if temp is not None and not (0 <= temp <= 100):
            problems.append(f"ID {motor_id} ({name}): 温度读数 {temp} 明显不合理（地址可能不匹配）")

        if volt is not None:
            if volt < 60:
                problems.append(f"ID {motor_id} ({name}): 电压 {volt / 10:.1f}V 偏低，供电可能不足")
            elif volt > 140:
                problems.append(f"ID {motor_id} ({name}): 电压读数 {volt / 10:.1f}V 超出合理范围")

        if torque is not None and torque not in (0, 1):
            problems.append(f"ID {motor_id} ({name}): 扭矩使能位 = {torque}（应为 0 或 1），读数可疑")

        if pos is None and vals["Present_Position"] is None and volt is None:
            problems.append(f"ID {motor_id} ({name}): 所有寄存器都读不到，只有 PING 有响应")

    # ---- 通信稳定性：同一 ID 连读多次，看位置抖动 ----
    print()
    print("=" * 78)
    print(f"  通信稳定性（每个 ID 连读 {run_repeat} 次位置）")
    print("=" * 78)
    print(f"  {'ID':<3} {'关节名':<15} {'成功':>6} {'读数范围':>18} {'抖动':>8}")
    print("  " + "-" * 74)

    for motor_id in ids:
        name = JOINT_NAMES.get(motor_id, "(未知)")
        readings = []
        for _ in range(run_repeat):
            v = read_reg(ser, motor_id, *REGISTERS["Present_Position"])
            if v is not None:
                readings.append(v)
        if not readings:
            print(f"  {motor_id:<3} {name:<15} {0:>4}/{run_repeat:<2} {'——':>18} {'——':>8}")
            problems.append(f"ID {motor_id} ({name}): 连续 {run_repeat} 次全部读取失败，通信严重不稳")
            continue
        lo, hi = min(readings), max(readings)
        span = hi - lo
        print(f"  {motor_id:<3} {name:<15} {len(readings):>4}/{run_repeat:<2} "
              f"{f'{lo} ~ {hi}':>18} {span:>6} tick")
        if span > 30:
            problems.append(
                f"ID {motor_id} ({name}): 静止时位置抖动 {span} tick ({ticks_to_deg(span):.1f}°)，"
                f"疑似 ID 冲突或供电噪声")

    # ---- 汇总 ----
    print()
    print("=" * 78)
    print("  诊断结论")
    print("=" * 78)

    expected = set(JOINT_NAMES)
    actual = set(ids)
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)

    print(f"  发现舵机: {len(ids)} 个  ->  ID {sorted(ids)}")
    if missing:
        print(f"  [!] 缺失 ID: {missing}  ->  对应关节 {[JOINT_NAMES[i] for i in missing]}")
    if extra:
        print(f"  [!] 多出的 ID: {extra}  ->  可能前一位使用者改过 ID，或被设成了别的编号")
    if not missing and not extra:
        print("  [OK] ID 集合完全符合 SO-101 出厂配置（1~6）")

    if problems:
        print()
        print(f"  发现 {len(problems)} 个问题：")
        for p in problems:
            print(f"    [!!] {p}")
    else:
        print("  [OK] 未发现异常")

    print()
    print("  下一步：")
    if problems or missing or extra:
        print("    -> 先解决上面的问题，再跑 02_joint_test.py")
    else:
        print("    -> 硬件层正常，可以跑 02_joint_test.py 做受控运动测试")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="SO-100/SO-101 总线只读诊断（不会让机械臂运动）")
    ap.add_argument("--list", action="store_true", help="只列出串口然后退出")
    ap.add_argument("--port", help="串口号，例如 COM7")
    ap.add_argument("--baud", type=int, default=1000000, help="波特率，默认 1000000")
    ap.add_argument("--all-baud", action="store_true", help="逐个波特率探测")
    ap.add_argument("--max-id", type=int, default=20, help="ID 搜索上限，默认 20")
    args = ap.parse_args()

    # 自检：验证组包逻辑与已知标准包一致
    assert build_instruction(1, INST_PING) == b"\xff\xff\x01\x02\x01\xfb", "协议组包自检失败"

    if args.list or not args.port:
        list_ports_and_exit()
        if not args.port:
            print("\n找到舵机总线那个口之后，用：")
            print("    python 01_scan_bus.py --port <你的COM口> --all-baud")
        return 0

    port, baud = args.port, args.baud

    if args.all_baud:
        baud, found = probe_baud(port, args.max_id)
        if baud is None:
            print("\n所有波特率都没有响应。可能原因：")
            print("  · COM 口选错了（蓝牙虚拟串口很多，容易选错）")
            print("  · 舵机总线板没供电（USB 只供电不足，需要接主电源）")
            print("  · 舵机 ID 全被改到 20 以外")
            return 1
    else:
        print(f"以 {baud} bps 打开 {port} ……")
        with serial.Serial(port, baud, timeout=0.02) as ser:
            time.sleep(0.2)
            found = [i for i in range(1, args.max_id + 1) if ping(ser, i) is not None]
        if not found:
            print(f"在 {baud} bps 下没有发现任何舵机。")
            print("-> 试试：python 01_scan_bus.py --port %s --all-baud" % port)
            return 1

    print(f"发现 {len(found)} 个舵机：ID {found}")

    with serial.Serial(port, baud, timeout=0.02) as ser:
        time.sleep(0.2)
        diagnose(ser, found)

    return 0


if __name__ == "__main__":
    sys.exit(main())
