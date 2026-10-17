#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SO-100 / SO-101 单关节受控运动测试

=========================================================================
【这个脚本会让机械臂动】—— 跑之前请确认：
  1. 机械臂放在空旷桌面，周围没有易碎物，线缆不会被带住
  2. 手放在电源开关或插头上，随时能断电
  3. 先跑过 01_scan_bus.py，且结论是「未发现异常」
  4. 知道要测哪个关节，一次只测一个，别贪多
=========================================================================

【安全机制】使能扭矩时不会让关节「弹」到别处：
  顺序是  读当前位置 -> 把目标位置写成当前位置 -> 再使能扭矩
  这样电机上电瞬间就锁在原地，而不是猛地冲向某个历史目标值。
  这个顺序是这套脚本存在的核心意义，别改成别的顺序。

【用法】
  # 先看要做什么，不动手
  python 02_joint_test.py --port COM7 --id 1 --dry-run

  # 真的小幅度动一下（默认 ±8 度）
  python 02_joint_test.py --port COM7 --id 1 --yes

  # 6 个关节依次测一遍
  python 02_joint_test.py --port COM7 --id all --yes

  # 更保守：只动 ±3 度
  python 02_joint_test.py --port COM7 --id 1 --delta 3 --yes
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行：  pip install pyserial")

# Windows 控制台/管道的编码可能是 GBK，编码不了的字符会直接抛异常中断脚本。
# 这个脚本正驱动着电机，中途崩溃很危险，所以必须先兜住打印这一层。
#
# 另外：直接连终端时保持系统编码（GBK），你自己看中文正常；
# 被重定向/管道捕获时（例如从 Claude Code 的 ! 里跑）改用 UTF-8，
# 否则接收方按 UTF-8 解码会得到一堆乱码。
for _stream in (sys.stdout, sys.stderr):
    try:
        if _stream.isatty():
            _stream.reconfigure(errors="replace")
        else:
            _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

JOINT_NAMES = {
    1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
    4: "wrist_flex", 5: "wrist_roll", 6: "gripper",
}

ADDR_TORQUE_ENABLE = 40      # 1 字节
ADDR_GOAL_POSITION = 42      # 2 字节
ADDR_PRESENT_POSITION = 56   # 2 字节
ADDR_PRESENT_LOAD = 60       # 2 字节
ADDR_PRESENT_TEMPERATURE = 63  # 1 字节
ADDR_PRESENT_CURRENT = 69    # 2 字节（部分固件不支持，读不到返回 None）
ADDR_GOAL_SPEED = 46         # 2 字节（可选，见 --speed 说明）
ADDR_MIN_POSITION_LIMIT = 9  # 2 字节，EEPROM 软限位下限（只读）
ADDR_MAX_POSITION_LIMIT = 11  # 2 字节，EEPROM 软限位上限（只读）

TICKS_PER_TURN = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_TURN

# 进入这个窗口就算"到位"，避免舵机差一两个 tick 时把循环卡死。
# 12 tick ≈ 1.05°，对 ±8° 的测试幅度来说足够宽松。
# 注意：这只是"判定"阈值，不是精度。稳定后的真实残差另行读取并打印。
ARRIVE_TOL = 12
SETTLE_READ_DELAY = 0.4   # 秒，进入容差窗口后再等这么久，读稳定值

# --- 动作等待策略 ---
# 不能用固定 deadline。要克服重力的关节（shoulder_lift / elbow_flex）走得更慢，
# 实测 ID 3 在 3 秒里只走完 85%，会被固定时间窗误判成"卡住"。
# 改成同时看两件事：进容差窗口 = 成功；位置连续多次不变 = 真的停了。
# 这样"慢但在动"和"卡住不动"就能分开。
MOVE_TIMEOUT = 8.0        # 秒，单个动作的硬上限
STALL_EPS = 2             # tick，位置变化小于这个算"没在动"
STALL_POLLS = 8           # 连续这么多次没动就认定停下（8 x 0.05s = 0.4s）


# ---------------------------------------------------------------- 协议

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


def write_reg(ser, motor_id: int, address: int, data: bytes) -> None:
    """写寄存器。写指令不等待应答，写完给舵机一点处理时间。"""
    ser.reset_input_buffer()
    ser.write(build_packet(motor_id, INST_WRITE, bytes([address]) + data))
    ser.flush()
    time.sleep(0.02)


def read_reg(ser, motor_id: int, address: int, length: int):
    ser.reset_input_buffer()
    ser.write(build_packet(motor_id, INST_READ, bytes([address, length])))
    pkt = read_status(ser)
    if pkt is None or pkt["id"] != motor_id or len(pkt["params"]) < length:
        return None
    return int.from_bytes(pkt["params"][:length], "little", signed=False)


def write_position(ser, motor_id: int, ticks: int) -> None:
    ticks = max(0, min(TICKS_PER_TURN - 1, int(ticks)))
    write_reg(ser, motor_id, ADDR_GOAL_POSITION, ticks.to_bytes(2, "little"))


def set_torque(ser, motor_id: int, enabled: bool) -> None:
    write_reg(ser, motor_id, ADDR_TORQUE_ENABLE, bytes([1 if enabled else 0]))


def decode_load(raw: int | None) -> int | None:
    if raw is None:
        return None
    magnitude = raw & 0x3FF
    return -magnitude if raw & 0x400 else magnitude


def ticks_to_deg(ticks: int) -> float:
    return ticks * DEG_PER_TICK


# ---------------------------------------------------------------- 单关节测试

def wait_arrive(ser, motor_id: int, target: int,
                timeout: float) -> tuple[int | None, int | None]:
    """
    等舵机走到 target 附近。

    返回 (到位时的位置, 最后读到的位置)。第一个是 None 就表示没到位。

    判定逻辑：
      - 进容差窗口           -> 成功
      - 位置连续 STALL_POLLS 次不变 -> 真的停下来了，没到位就是真失败
      - 到 timeout           -> 超时，也算失败

    不用"固定时间到没到"来判定，是因为要克服重力的关节走得慢，
    可能还在收敛就被判成卡住。
    """
    deadline = time.time() + timeout
    last = None
    stall = 0
    while time.time() < deadline:
        cur = read_reg(ser, motor_id, ADDR_PRESENT_POSITION, 2)
        if cur is not None:
            if abs(cur - target) <= ARRIVE_TOL:
                return cur, cur
            if last is not None and abs(cur - last) < STALL_EPS:
                stall += 1
                if stall >= STALL_POLLS:
                    return None, cur
            else:
                stall = 0
            last = cur
        time.sleep(0.05)
    return None, last


def test_one_joint(ser, motor_id: int, delta_deg: float, settle: float,
                   speed: int | None, dry_run: bool) -> bool:
    name = JOINT_NAMES.get(motor_id, "(未知)")
    delta_ticks = int(round(delta_deg / DEG_PER_TICK))

    print()
    print("-" * 70)
    print(f"  关节 ID {motor_id}  ({name})")
    print("-" * 70)

    ser.reset_input_buffer()
    start_raw = read_reg(ser, motor_id, ADDR_PRESENT_POSITION, 2)
    if start_raw is None:
        print("  [!!] 读不到当前位置，跳过（先解决通信问题）")
        return False
    temp_before = read_reg(ser, motor_id, ADDR_PRESENT_TEMPERATURE, 1)

    # --- 读软限位 ---
    # 舵机会直接忽略超出 [min, max] 的目标值，表现为「一个 tick 都不动」，
    # 看着像坏了，其实是正常保护。所以必须先读出来，别朝顶死的方向发目标。
    # 这两个是 EEPROM 寄存器，地址见 lerobot/motors/feetech/tables.py。
    mn = read_reg(ser, motor_id, ADDR_MIN_POSITION_LIMIT, 2)
    mx = read_reg(ser, motor_id, ADDR_MAX_POSITION_LIMIT, 2)

    print(f"  起始位置: {start_raw} tick  ({ticks_to_deg(start_raw):.1f}°)")
    if temp_before is not None:
        print(f"  起始温度: {temp_before}°C")

    if mn is not None and mx is not None:
        print(f"  软限位:   {mn} ~ {mx}   (跨度 {(mx - mn) * DEG_PER_TICK:.1f}°)")
    else:
        print("  [!] 读不到软限位，按 0~4095 处理")

    # 起始位置本身可能落在限位之外（前任使用者留下的姿态）。
    # 那样一使能扭矩，舵机会自己爬回合法区间，制造出「漂移」的假象 ——
    # 上一轮 ID 4 的 +10 tick 漂移就是这么来的（起点 874，下限 889）。
    # 先夹进合法区间，后面的判断才准。
    start = start_raw
    if mn is not None and start < mn:
        print(f"  [!] 起始位置低于下限 {mn}，夹到下限（否则舵机会自己爬回去）")
        start = mn
    elif mx is not None and start > mx:
        print(f"  [!] 起始位置高于上限 {mx}，夹到上限")
        start = mx

    # --- 选一个有行程余量的方向 ---
    # 默认正向；正向顶到上限就自动改测反向，两边都顶死才算真失败。
    target = start + delta_ticks
    direction = 1
    if mx is not None and target > mx:
        if mn is not None and (start - delta_ticks) >= mn:
            print(f"  [!] 正向目标 {target} 超过上限 {mx}，自动改测反向")
            direction = -1
            target = start - delta_ticks
        else:
            print(f"  [!!] 正向目标 {target} 超过上限 {mx}，反向又低于下限 {mn}")
            print("      -> 该关节在两个方向都没有行程余量，跳过")
            return False
    if not (0 <= target < TICKS_PER_TURN):
        print(f"  [!!] 目标位置 {target} 超出 0~4095，跳过")
        return False
    if mn is not None and target < mn:
        print(f"  [!!] 反向目标 {target} 低于下限 {mn}，跳过")
        return False

    delta_deg = direction * abs(delta_deg)
    print(f"  计划动作: {start} -> {target}  ({delta_deg:+.1f}° -> 再回到原位)")
    print(f"  安全顺序: 先写目标=当前位置 -> 使能扭矩 -> 移动 -> 回位 -> 关扭矩")

    if dry_run:
        print("  [dry-run] 未执行任何写操作")
        return True

    if speed is not None:
        print(f"  设置运动速度: {speed}（寄存器 {ADDR_GOAL_SPEED}）")
        write_reg(ser, motor_id, ADDR_GOAL_SPEED, int(speed).to_bytes(2, "little"))

    # --- 关键安全步骤 1: 扭矩还关着的时候，先把目标位置设成当前位置 ---
    write_position(ser, motor_id, start)

    # --- 关键安全步骤 2: 现在使能扭矩，电机锁在原地不会弹 ---
    set_torque(ser, motor_id, True)
    time.sleep(settle)
    held = read_reg(ser, motor_id, ADDR_PRESENT_POSITION, 2)
    if held is not None:
        drift = held - start
        arrow = "[OK]" if abs(drift) <= 15 else "[!]"
        print(f"  {arrow} 使能扭矩后位置 {held} tick（相对起始漂移 {drift:+d} tick）")
        if abs(drift) > 15:
            print("    漂移偏大：可能目标寄存器没写进去，或该关节负载很重")

    ok = True
    try:
        # --- 移动到目标 ---
        write_position(ser, motor_id, target)
        reached, last = wait_arrive(ser, motor_id, target, MOVE_TIMEOUT)

        if reached is None:
            gap = None if last is None else last - target
            print(f"  [!!] 未能到达目标位置（{delta_deg:+.1f}°），停在 {last}")
            if gap is not None:
                print(f"       离目标还差 {gap:+d} tick ({gap * DEG_PER_TICK:+.2f}°)，"
                      f"且已停止移动")
            # 停住瞬间的负载和电流，是区分"顶不动"和"放弃不出力"的关键：
            #   负载/电流大 -> 在使劲顶，有东西挡着（重力/干涉/卡涩）
            #   负载/电流接近 0 -> 舵机压根没出力（配置或驱动问题）
            load = decode_load(read_reg(ser, motor_id, ADDR_PRESENT_LOAD, 2))
            cur_draw = read_reg(ser, motor_id, ADDR_PRESENT_CURRENT, 2)
            held_pos = read_reg(ser, motor_id, ADDR_PRESENT_POSITION, 2)
            print(f"       停住时的负载 {load}   电流 {cur_draw}   "
                  f"位置 {held_pos}")
            print("    判读：负载/电流大 = 在使劲顶（有阻挡）；接近 0 = 没出力")
            ok = False
        else:
            # 上面的循环一进容差窗口就退出了，此时舵机可能还在微调。
            # 再等一小会儿读"稳定值"，这才是真正的到位精度。
            time.sleep(SETTLE_READ_DELAY)
            settled = read_reg(ser, motor_id, ADDR_PRESENT_POSITION, 2)
            if settled is None:
                settled = reached
            load = decode_load(read_reg(ser, motor_id, ADDR_PRESENT_LOAD, 2))
            err = settled - target
            print(f"  [OK] 正向移动到位: {settled} tick  负载 {load}"
                  f"  残差 {err:+d} tick ({err * DEG_PER_TICK:+.2f}°)")

        # --- 反向移动，回到原位 ---
        write_position(ser, motor_id, start)
        back, last_back = wait_arrive(ser, motor_id, start, MOVE_TIMEOUT)
        if back is None:
            print(f"  [!!] 未能反向回到原位，停在 {last_back}")
            ok = False
        else:
            time.sleep(SETTLE_READ_DELAY)
            settled_back = read_reg(ser, motor_id, ADDR_PRESENT_POSITION, 2)
            if settled_back is None:
                settled_back = back
            err = settled_back - start
            print(f"  [OK] 反向回到原位: {settled_back} tick"
                  f"  残差 {err:+d} tick ({err * DEG_PER_TICK:+.2f}°)")

        temp_after = read_reg(ser, motor_id, ADDR_PRESENT_TEMPERATURE, 1)
        if temp_before is not None and temp_after is not None:
            print(f"  温度: {temp_before}°C -> {temp_after}°C")

    finally:
        # --- 无论如何都要松开扭矩，别让电机一直憋着 ---
        set_torque(ser, motor_id, False)
        print("  已关闭扭矩，关节现在可以用手自由扳动")

    return ok


# ---------------------------------------------------------------- 主流程

def main() -> int:
    ap = argparse.ArgumentParser(description="SO-101 单关节受控运动测试（会让机械臂动）")
    ap.add_argument("--port", required=True, help="串口号，例如 COM7")
    ap.add_argument("--baud", type=int, default=1000000, help="波特率，默认 1000000")
    ap.add_argument("--id", required=True,
                    help="关节 ID（1~6），或 all 表示按顺序全测")
    ap.add_argument("--delta", type=float, default=8.0, help="单次移动角度，默认 8 度")
    ap.add_argument("--settle", type=float, default=0.3, help="动作后等待时间，秒")
    ap.add_argument("--speed", type=int, default=None,
                    help="运动速度上限。默认不写该寄存器（更保守）。"
                         "注意：不同固件此寄存器地址可能不同，写错地址有风险，"
                         "不确定就别传这个参数。")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不执行任何写操作")
    ap.add_argument("--yes", action="store_true",
                    help="确认你已把机械臂放在安全位置、手放在电源开关上")
    args = ap.parse_args()

    assert build_packet(1, INST_PING) == b"\xff\xff\x01\x02\x01\xfb", "协议组包自检失败"

    if not args.dry_run and not args.yes:
        print("拒绝执行：这是会驱动电机的脚本。")
        print()
        print("请先确认三件事：")
        print("  1. 机械臂已固定在桌面，周围清空，线缆不会被带住")
        print("  2. 你的手在电源开关上，随时能断电")
        print("  3. 已经跑过 01_scan_bus.py 且没有报警")
        print()
        print("确认后再加 --yes 重跑。想先看计划就用 --dry-run。")
        return 2

    if args.delta <= 0 or args.delta > 30:
        sys.exit("--delta 请设在 0~30 度之间。首次测试建议 5~8 度。")

    if args.id.lower() == "all":
        ids = [1, 2, 3, 4, 5, 6]
    else:
        try:
            ids = [int(args.id)]
        except ValueError:
            sys.exit("--id 需要是 1~6 的整数，或者 all")
        if not 1 <= ids[0] <= 6:
            sys.exit("--id 需要在 1~6 范围内")

    print("=" * 70)
    print("  SO-101 单关节受控运动测试")
    print("=" * 70)
    print(f"  端口 {args.port} @ {args.baud} bps")
    print(f"  测试关节: {[(i, JOINT_NAMES[i]) for i in ids]}")
    print(f"  单次幅度: ±{args.delta}°   模式: {'dry-run（不动）' if args.dry_run else '实际执行'}")

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        time.sleep(0.2)
        results = {}
        for i in ids:
            results[i] = test_one_joint(ser, i, args.delta, args.settle,
                                        args.speed, args.dry_run)
            if i != ids[-1]:
                print("\n  3 秒后测下一个关节……（Ctrl-C 可随时中止）")
                time.sleep(3)

    if args.dry_run:
        print("\n  dry-run 结束，未发生任何写入。去掉 --dry-run 并加 --yes 即可真跑。")
        return 0

    print()
    print("=" * 70)
    print("  汇总")
    print("=" * 70)
    for i in ids:
        mark = "[OK] 通过" if results.get(i) else "[!!] 异常"
        print(f"  ID {i:<2} {JOINT_NAMES[i]:<15} {mark}")

    n_bad = sum(1 for v in results.values() if not v)
    if n_bad:
        print(f"\n  {n_bad} 个关节有问题，先排查再往下走。")
        return 1
    print("\n  全部通过。可以继续到遥操作测试（见 README.md 第 4 步）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
