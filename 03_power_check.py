#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
供电诊断：判断电压读数是"真实的 ADC 采样"还是"读错了寄存器"

【原理】
  真实电压由 ADC 采样，即使稳压电源也会有 ±0.1V 级的微小波动，
  连读几十次一定能看到数值变化。
  如果读的是配置寄存器（地址错了），那么每次读出来都是同一个常数，
  一个 bit 都不会变 —— 这是区分两者的决定性证据。

  本脚本还会在读数完全不变时给出明确警告。

【安全】只读，不发送任何写指令，机械臂不会动。

【用法】
  python 03_power_check.py --port COM23
  python 03_power_check.py --port COM23 --samples 80
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：  pip install pyserial")

# 直接连终端时保持系统编码（GBK），你自己看中文正常；
# 被重定向/管道捕获时改用 UTF-8，否则接收方按 UTF-8 解码会得到一堆乱码。
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

JOINT_NAMES = {
    1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
    4: "wrist_flex", 5: "wrist_roll", 6: "gripper",
}

ADDR_PRESENT_VOLTAGE = 62      # 0.1V 单位
ADDR_PRESENT_TEMPERATURE = 63  # °C
ADDR_MAX_VOLTAGE_LIMIT = 14    # 0.1V 单位，舵机出厂设定的允许上限
ADDR_MIN_VOLTAGE_LIMIT = 15    # 0.1V 单位，舵机出厂设定的允许下限
#
# 地址 62 已由 lerobot 官方寄存器表交叉验证：
#   lerobot/motors/feetech/tables.py:  "Present_Voltage": (62, 1)
# 所以"读出来是常数 == 地址读错了"这个推断是不成立的，
# 见本文件末尾结论部分的判断逻辑。


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


def ping(ser, motor_id: int):
    ser.reset_input_buffer()
    ser.write(build_packet(motor_id, INST_PING))
    pkt = read_status(ser)
    return pkt if (pkt and pkt["id"] == motor_id) else None


# 状态包里的 error 字节是舵机自己的故障诊断
ERROR_BITS = {
    0x01: "电压异常",
    0x02: "角度/传感器异常",
    0x04: "过温",
    0x08: "过流",
    0x10: "负载/堵转",
    0x20: "指令错误",
}


def main() -> int:
    ap = argparse.ArgumentParser(description="供电诊断（只读，不会让机械臂运动）")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--samples", type=int, default=50, help="每个关节采样次数")
    args = ap.parse_args()

    assert build_packet(1, INST_PING) == b"\xff\xff\x01\x02\x01\xfb", "协议自检失败"

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        time.sleep(0.2)
        ids = [i for i in range(1, 7) if ping(ser, i) is not None]
        if not ids:
            print("[!!] 没有发现舵机，检查端口和波特率")
            return 1

        print("=" * 74)
        print(f"  供电稳定性诊断（每关节采样 {args.samples} 次）")
        print("=" * 74)

        # ---- 1. 电压连续性 ----
        print()
        print(f"  {'ID':<3} {'关节名':<15} {'最小':>7} {'最大':>7} {'均值':>7} "
              f"{'不同值':>7} {'结论':>10}")
        print("  " + "-" * 70)

        all_constant = True
        all_jitter = True
        voltages: dict[int, list[int]] = {}

        for motor_id in ids:
            name = JOINT_NAMES.get(motor_id, "(未知)")
            vals = []
            for _ in range(args.samples):
                v = read_reg(ser, motor_id, ADDR_PRESENT_VOLTAGE, 1)
                if v is not None:
                    vals.append(v)
                time.sleep(0.01)
            voltages[motor_id] = vals
            if not vals:
                print(f"  {motor_id:<3} {name:<15} {'——':>7} {'——':>7} {'——':>7} "
                      f"{'——':>7} {'读取失败':>10}")
                continue
            distinct = len(set(vals))
            if distinct > 1:
                all_constant = False
                verdict = "ADC 真实"
            else:
                all_jitter = False
                verdict = "疑似常数"
            print(f"  {motor_id:<3} {name:<15} "
                  f"{min(vals) / 10:>6.1f}V {max(vals) / 10:>6.1f}V {sum(vals) / len(vals) / 10:>6.2f}V "
                  f"{distinct:>7} {verdict:>10}")

        # ---- 2. 温度连续性（作为对照组）----
        print()
        print("  对照组：温度读数（温度变化更慢，抖动应小于电压）")
        print(f"  {'ID':<3} {'关节名':<15} {'最小':>7} {'最大':>7} {'不同值':>7}")
        print("  " + "-" * 50)
        temp_any_jitter = False
        for motor_id in ids:
            name = JOINT_NAMES.get(motor_id, "(未知)")
            vals = []
            for _ in range(20):
                v = read_reg(ser, motor_id, ADDR_PRESENT_TEMPERATURE, 1)
                if v is not None:
                    vals.append(v)
                time.sleep(0.01)
            if vals:
                if len(set(vals)) > 1:
                    temp_any_jitter = True
                print(f"  {motor_id:<3} {name:<15} {min(vals):>6}°C {max(vals):>6}°C "
                      f"{len(set(vals)):>7}")

        # ---- 时钟机的出厂允许电压窗口 ----
        print()
        print("  舵机出厂设定的电压允许窗口（EEPROM 地址 14/15）")
        print(f"  {'ID':<3} {'关节名':<15} {'下限':>8} {'上限':>8} {'当前':>8} {'判断'}")
        print("  " + "-" * 62)
        win = {}
        for motor_id in ids:
            name = JOINT_NAMES.get(motor_id, "(未知)")
            lo = read_reg(ser, motor_id, ADDR_MIN_VOLTAGE_LIMIT, 1)
            hi = read_reg(ser, motor_id, ADDR_MAX_VOLTAGE_LIMIT, 1)
            cur = voltages.get(motor_id) or []
            cur = sum(cur) / len(cur) if cur else None
            if lo is None or hi is None:
                print(f"  {motor_id:<3} {name:<15} {'——':>8} {'——':>8} {'——':>8}  读不到")
                continue
            win[motor_id] = (lo, hi)
            ok = cur is not None and lo <= cur <= hi
            print(f"  {motor_id:<3} {name:<15} {lo / 10:>7.1f}V {hi / 10:>7.1f}V "
                  f"{(cur / 10 if cur else 0):>7.1f}V   {'OK' if ok else '超出出厂窗口!'}")

        # ---- 3. 舵机自己的故障标志 ----
        print()
        print("  舵机自报故障标志（状态包的 error 字节）")
        print("  " + "-" * 50)
        any_error = False
        for motor_id in ids:
            ser.reset_input_buffer()
            ser.write(build_packet(motor_id, INST_PING))
            pkt = read_status(ser)
            err = pkt["error"] if pkt else None
            if err is None:
                continue
            if err == 0:
                print(f"  ID {motor_id} ({JOINT_NAMES.get(motor_id, '?'):<14}) 无故障")
            else:
                any_error = True
                names = [n for bit, n in ERROR_BITS.items() if err & bit]
                print(f"  ID {motor_id} ({JOINT_NAMES.get(motor_id, '?'):<14}) "
                      f"error=0x{err:02X}  {' + '.join(names) or '未知位'}")
        if not any_error:
            print()
            print("  所有舵机都没有报告电压/过流/过温故障。")

        # ---- 4. 判断 ----
        print()
        print("=" * 74)
        print("  结论")
        print("=" * 74)
        # 判断"读数恒定"到底是电源稳，还是地址读错了。
        # 决定性区分：温度做对照组。温度在同一批采样里会跳，说明 ADC / 读取
        # 链路是活的；那电压恒定就只能解释成电源真的很稳。
        # 另外地址 62 已被 lerobot 官方表确认，所以不该再往"地址错"上靠。
        if all_constant and not temp_any_jitter:
            print("  [!] 电压和温度在整批采样里 bit 级完全不变。")
            print("      整条读取链路可疑（不只是电压），先确认端口/波特率/接线。")
        elif all_constant:
            print("  [OK] 电压读数恒定，但同一批采样里温度在跳 ——")
            print("       说明 ADC 与读取链路正常，电压恒定是电源本身稳，不是读错寄存器。")
            print("       （地址 62 已与 lerobot 官方寄存器表交叉验证一致）")
        else:
            sample = next((v for v in voltages.values() if len(set(v)) > 1), [])
            print(f"  [OK] 电压读数有波动（例如 {min(sample) / 10:.1f} ~ "
                  f"{max(sample) / 10:.1f}V），确实在读实时电压。")

        if voltages:
            lo = min(min(v) for v in voltages.values() if v)
            hi = max(max(v) for v in voltages.values() if v)
            print(f"       实测范围 {lo / 10:.1f} ~ {hi / 10:.1f}V")

        # 最终判断以**舵机自己的出厂窗口**为准，而不是 STS3215 的通用标称值
        print()
        if win and voltages:
            out_of_window = []
            for motor_id, (wlo, whi) in win.items():
                cur = voltages.get(motor_id) or []
                if not cur:
                    continue
                avg = sum(cur) / len(cur)
                if not (wlo <= avg <= whi):
                    out_of_window.append((motor_id, avg, wlo, whi))
            wlo = min(w[0] for w in win.values()) / 10
            whi = max(w[1] for w in win.values()) / 10
            if out_of_window:
                print(f"  [!!] 有 {len(out_of_window)} 个舵机的电压超出自己的出厂窗口：")
                for motor_id, avg, l, h in out_of_window:
                    print(f"       ID {motor_id}: {avg / 10:.1f}V 不在 "
                          f"{l / 10:.1f}~{h / 10:.1f}V 内")
            else:
                print(f"  [OK] 所有关节电压都落在舵机出厂窗口 {wlo:.1f}~{whi:.1f}V 内。")
                print("       供电正常，可以开始运动测试。")
        else:
            print("  [!] 读不到舵机的电压窗口，无法判断。")
            print("       STS3215 通用标称是 6~8.4V，可据此人工确认。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
