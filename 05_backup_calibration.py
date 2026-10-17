#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
校准备份 / 恢复 / 安装（主从臂各跑一次）

=========================================================================
【为什么需要这个脚本】

`lerobot-calibrate` 不是"填个表"，它会**直接往舵机 EEPROM 里写**三个寄存器
（源码：lerobot/motors/feetech/feetech.py 的 write_calibration）：

    Homing_Offset       (地址 31)   零点偏移
    Min_Position_Limit  (地址  9)   最小软限位
    Max_Position_Limit  (地址 11)   最大软限位

这副臂是二手的，**前任已经校准过**——除了 wrist_roll，每个关节的软限位都收窄
到了 193~212° 的合理行程（见 04_limits_check.py）。一旦跑 lerobot-calibrate，
这些值就被冲掉，回不去了。

所以：**先备份，再校准。**

【两种用法】

  A. 只备份（先做这个，只读，机械臂不会动）
       python 05_backup_calibration.py --port COM23 --arm follower --id follower1
       python 05_backup_calibration.py --port COM32 --arm leader   --id leader1

  B. 干脆不跑 lerobot-calibrate —— 直接把 EEPROM 里的前任校准
     转成 lerobot 的 calibration JSON 装上（不碰 EEPROM）
       python 05_backup_calibration.py --port COM23 --arm follower --id follower1 --install

     lerobot 启动时判断"校准好没好"用的是 bus.is_calibrated，
     它的逻辑是：**文件里的值 == 舵机 EEPROM 里的值** → 就算已校准。
     所以从 EEPROM 反向生成的文件能直接通过，不用重写任何东西。

  C. 万一已经被冲掉了，从备份写回 EEPROM
       python 05_backup_calibration.py --port COM23 --arm follower --id follower1 \
              --restore calib_backup/follower_follower1_20261017_153000.json --yes

  D. 修「lerobot 说发现 0 个舵机」（主臂的实况，见下面【电压窗口】）
       python 05_backup_calibration.py --port COM32 --arm leader --id leader1 \
              --voltage-window 40 140 --yes

=========================================================================
【电压窗口：主臂为什么用不了 lerobot】

2026-10-17 实测：主臂（COM32）每个舵机的应答包都带 `0x01 输入电压错误`。
原因不是硬件，是舵机 EEPROM 里的电压保护窗口被设窄了：

    从臂 COM23：允许 40~140 (4.0~14.0V)，实际 12.5V  -> 无错误
    主臂 COM32：允许 40~120 (4.0~12.0V)，实际 12.5V  -> 超上限
                其中 ID 2 是 40~80 (4.0~8.0V)         -> 超得离谱

lerobot 的 `_assert_motors_exist` 会把**任何带错误标志的舵机整个过滤掉**，
于是握手时 `Full found motor list: {}`，报「Missing motor IDs: 1~6」。
`lerobot-calibrate` / `lerobot-teleoperate` / `lerobot-record` 全都起不来。

裸协议不受影响（错误标志只是应答里的一个字节，不影响数据本身），
所以本脚本会**自动退回裸协议**，保证备份/恢复始终能用。

修法就是把窗口设回标准值 `40 140`，和从臂一致。


=========================================================================
【安全说明】

  · 默认模式（备份）和 --install **只读舵机**，不写任何寄存器
  · --restore 会写 EEPROM，必须显式加 --yes
  · 本脚本**不主动开关扭矩**。因为关扭矩 = 机械臂立刻失去支撑，
    可能直接砸下来。要不要关由你自己决定（先用手托住或把臂放低）
  · 退出时不做任何状态改动
=========================================================================
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

# 中文输出：管道里强制 utf-8，否则 Claude Code 侧会按 utf-8 解码 gbk 字节 → 乱码
for _s in (sys.stdout, sys.stderr):
    try:
        if _s.isatty():
            _s.reconfigure(errors="replace")
        else:
            _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    from lerobot.motors.encoding_utils import decode_sign_magnitude, encode_sign_magnitude
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorCalibration, MotorNormMode
    from lerobot.utils.constants import (
        HF_LEROBOT_CALIBRATION,
        ROBOTS,
        TELEOPERATORS,
    )
except ImportError as e:
    sys.exit(
        f"导入 lerobot 失败：{e}\n"
        "请用这个 venv 的解释器运行：\n"
        "  E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py ..."
    )


HERE = Path(__file__).resolve().parent
BACKUP_DIR = HERE / "calib_backup"

# 两个臂的电机表完全一样。
# 抄自 lerobot/robots/so_follower/so_follower.py:53-60 和
#      lerobot/teleoperators/so_leader/so_leader.py:45-52
# norm_mode 对 read_calibration 无影响（那边用 normalize=False），
# 但为了和数据采集时一致，这里照抄成 DEGREES / RANGE_0_100。
def so101_motors() -> dict[str, Motor]:
    body = MotorNormMode.DEGREES
    return {
        "shoulder_pan": Motor(1, "sts3215", body),
        "shoulder_lift": Motor(2, "sts3215", body),
        "elbow_flex": Motor(3, "sts3215", body),
        "wrist_flex": Motor(4, "sts3215", body),
        "wrist_roll": Motor(5, "sts3215", body),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    }


# 备份时顺手多读几个只读寄存器，方便日后回看"备份当时的臂是什么状态"。
# 电压上下限一定要备 —— 主臂就是因为这两个值被设窄了才用不了 lerobot。
EXTRA_REGS = [
    "Present_Position",
    "Present_Voltage",
    "Present_Temperature",
    "Min_Voltage_Limit",
    "Max_Voltage_Limit",
    "Max_Torque_Limit",
    "Torque_Limit",
    "Torque_Enable",
]

# STS3215 的标准出厂电压窗口（从臂就是这个值，见 ENVIRONMENT.md 第 1 节）。
DEFAULT_VOLTAGE_WINDOW = (40, 140)  # 4.0V ~ 14.0V

DEG_PER_TICK = 360.0 / 4096


class RawBus:
    """裸串口版的总线客户端，接口对齐 FeetechMotorsBus 的那几个方法。

    【为什么需要它】
    lerobot 的 connect() 会做握手，而 _assert_motors_exist 会把**任何带错误标志
    的舵机直接过滤掉**。主臂（COM32）每个舵机都在报"输入电压错误"
    （Max_Voltage_Limit 被设成 12.0V，而供电是 12.5V），于是 lerobot 认为
    "没找到舵机"，连备份都做不了 —— 那就连回滚的机会都没有了。

    报错的是**应答包里的错误字节**，不是通信本身。裸协议能正常读到数据，
    所以这条路是通的。它是只读 + 需要时写入的安全网，不依赖任何 lerobot 代码。
    """

    # (地址, 字节数, 符号位下标 或 None)
    #
    # 【重要】Homing_Offset 不是二进制补码，是 **sign-magnitude**：
    #   bit 11 = 符号位，bit 0~10 = 幅值。
    #   实测对照（COM23，lerobot vs 裸协议）：
    #       原始 2406 -> 2406 & 0x7FF = 358，bit11 置位 -> -358
    #       原始 2117 -> -69      原始 2071 -> -23
    #   按补码解会得到 2406 / 2117 / 2071，正数碰巧对、负数全错 ——
    #   然后 --restore 就会把这几个垃圾值写回舵机。
    #   符号位下标来自 lerobot 的 STS_SMS_SERIES_ENCODINGS_TABLE。
    REGS = {
        "Min_Position_Limit": (9, 2, None),      # 无符号
        "Max_Position_Limit": (11, 2, None),     # 无符号
        "Max_Voltage_Limit": (14, 1, None),      # 无符号
        "Min_Voltage_Limit": (15, 1, None),      # 无符号
        "Max_Torque_Limit": (16, 2, None),       # 无符号
        "Homing_Offset": (31, 2, 11),            # sign-magnitude，符号位 bit 11
        "Torque_Enable": (40, 1, None),          # 无符号
        "Torque_Limit": (48, 2, None),           # 无符号
        "Lock": (55, 1, None),                   # 无符号
        "Present_Position": (56, 2, 15),         # lerobot 表里符号位是 15；
                                                 # 位置只到 4095，bit15 永远为 0，
                                                 # 结果与无符号一致，此处照抄以防万一
        "Present_Voltage": (62, 1, None),        # 无符号
        "Present_Temperature": (63, 1, None),    # 无符号
    }

    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03

    def __init__(self, port: str, motors: dict[str, Motor], baudrate: int = 1000000):
        import serial  # 局部导入：只有走裸协议时才需要

        self._serial = serial
        self.port = port
        self.motors = motors
        self.baudrate = baudrate
        self.ser = None

    # ---------- 协议 ----------

    @staticmethod
    def _checksum(payload: bytes) -> int:
        return (~sum(payload)) & 0xFF

    def _packet(self, motor_id: int, instruction: int, params: bytes = b"") -> bytes:
        body = bytes([motor_id, len(params) + 2, instruction]) + bytes(params)
        return b"\xff\xff" + body + bytes([self._checksum(body)])

    def _read_status(self, timeout: float = 0.05):
        deadline = time.time() + timeout
        seen = 0
        while time.time() < deadline:
            b = self.ser.read(1)
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
        rest = self.ser.read(2)
        if len(rest) < 2:
            return None
        motor_id, length = rest[0], rest[1]
        if length < 2:
            return None
        payload = self.ser.read(length)
        if len(payload) < length:
            return None
        body, chk = payload[:-1], payload[-1]
        if self._checksum(bytes([motor_id, length]) + body) != chk:
            return None
        return {"id": motor_id, "error": body[0], "params": body[1:]}

    def _read_raw(self, motor_id: int, addr: int, length: int):
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(motor_id, self.INST_READ, bytes([addr, length])))
        pkt = self._read_status()
        if pkt is None or pkt["id"] != motor_id or len(pkt["params"]) < length:
            return None
        raw = int.from_bytes(pkt["params"][:length], "little", signed=False)
        return raw, pkt["error"]

    # ---------- 对齐 FeetechMotorsBus 的接口 ----------

    def connect(self, handshake: bool = True) -> None:
        self.ser = self._serial.Serial(self.port, self.baudrate, timeout=0.02)
        time.sleep(0.2)
        if handshake:
            missing = [i for i, m in self.motors.items()
                       if self._read_raw(m.id, *self.REGS["Present_Position"][:2]) is None]
            if missing:
                self.ser.close()
                raise ConnectionError(f"端口 {self.port} 上这些关节没有应答：{missing}")

    def disconnect(self, disable_torque: bool = False) -> None:
        # 故意不开关扭矩：关扭矩 = 机械臂立刻失去支撑
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def read(self, reg: str, motor: str, normalize: bool = False) -> int:
        addr, length, sign_bit = self.REGS[reg]
        got = self._read_raw(self.motors[motor].id, addr, length)
        if got is None:
            raise IOError(f"{motor} 的 {reg} 读取失败")
        raw = got[0]
        return decode_sign_magnitude(raw, sign_bit) if sign_bit is not None else raw

    def write(self, reg: str, motor: str, value: int) -> None:
        addr, length, sign_bit = self.REGS[reg]
        if sign_bit is not None:
            value = encode_sign_magnitude(int(value), sign_bit)
        params = bytes([addr]) + int(value).to_bytes(length, "little", signed=False)
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(self.motors[motor].id, self.INST_WRITE, params))
        pkt = self._read_status()
        if pkt is None or pkt["id"] != self.motors[motor].id:
            raise IOError(f"{motor} 的 {reg} 写入没有收到应答")

    def read_calibration(self) -> dict[str, MotorCalibration]:
        cal = {}
        for name, m in self.motors.items():
            cal[name] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=int(self.read("Homing_Offset", name)),
                range_min=int(self.read("Min_Position_Limit", name)),
                range_max=int(self.read("Max_Position_Limit", name)),
            )
        return cal

    # 报错字节解码，供诊断打印用
    ERR_BITS = [
        (0x01, "输入电压错误"), (0x02, "角度限位错误"), (0x04, "过热"),
        (0x08, "超范围"), (0x10, "校验和错误"), (0x20, "过载"), (0x40, "指令错误"),
    ]

    def ping_errors(self) -> dict[str, tuple[int, str]]:
        """返回 {关节名: (错误字节, 文字)}。只发 PING。"""
        out = {}
        for name, m in self.motors.items():
            self.ser.reset_input_buffer()
            self.ser.write(self._packet(m.id, self.INST_PING))
            pkt = self._read_status()
            if pkt is None:
                out[name] = (-1, "无应答")
                continue
            err = pkt["error"]
            desc = "无错误" if err == 0 else " | ".join(
                n for bit, n in self.ERR_BITS if err & bit) or f"未知 0x{err:02X}"
            out[name] = (err, desc)
        return out


def make_bus(args) -> tuple[object, bool]:
    """返回 (bus, 是否为裸协议)。

    --raw 强制走裸协议；否则先用 lerobot 的 bus，握手失败就自动退回裸协议
    （主臂的电压错误标志会让 lerobot 的握手失败，但它本身通信是好的）。
    """
    if args.raw:
        print("  [i] 按 --raw 走裸串口协议\n")
        return RawBus(args.port, so101_motors()), True

    bus = FeetechMotorsBus(port=args.port, motors=so101_motors())
    try:
        bus.connect()
    except Exception as e:
        msg = str(e).splitlines()
        print("  [!] lerobot 的握手失败：")
        for line in msg[:4]:
            print(f"        {line}")
        # 关键：connect() 里 _connect 已经把串口打开了，握手抛异常时不会自动关。
        # 不关掉的话下面的 RawBus 会撞上 PermissionError(13, 拒绝访问)。
        try:
            if bus.port_handler.is_open:
                bus.port_handler.closePort()
        except Exception:
            pass
        print("      自动退回裸串口协议（通信本身是好的，是错误标志把 lerobot 挡住的）\n")
        return RawBus(args.port, so101_motors()), True

    # 探通了就断开，把 connect() 留给调用方 —— 否则返回的是「已连接」状态，
    # 调用方再 connect() 会抛 DeviceAlreadyConnectedError。
    # disable_torque=False：lerobot 默认会在断开时写 Torque_Enable=0，
    # 那是**写操作**，而且会让机械臂失去支撑。这个工具不碰扭矩。
    bus.disconnect(disable_torque=False)
    return bus, False


def calibration_target(arm: str, arm_id: str, port: str) -> Path:
    """lerobot 期望的 calibration JSON 路径。

    源码依据：lerobot/robots/robot.py:47-53
        calibration_dir   = config.calibration_dir or HF_LEROBOT_CALIBRATION / ROBOTS / self.name
        calibration_fpath = calibration_dir / f"{self.id}.json"

    **注意 self.name 不是类型名**，而是类里的 name 属性：
        SOFollower.name = "so_follower"    SOLeader.name = "so_leader"
    所以真实路径比想当然的多一层：
        .../calibration/robots/so_follower/follower1.json
        .../calibration/teleoperators/so_leader/leader1.json
    早期按 .../robots/follower1.json 写过一次，lerobot 根本找不到文件，
    is_calibrated 于是返回 False 并直接进了交互式 calibrate()。

    这里**不硬编码路径**，直接构造真类让它自己算 —— 免得 lerobot 改版后再错一次。
    构造对象不会打开串口（打开串口的是 connect()）。
    """
    if arm == "follower":
        from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
        from lerobot.robots.so_follower.so_follower import SO101Follower

        return SO101Follower(SO101FollowerConfig(port=port, id=arm_id)).calibration_fpath

    from lerobot.teleoperators.so_leader.config_so_leader import SO101LeaderConfig
    from lerobot.teleoperators.so_leader.so_leader import SO101Leader

    return SO101Leader(SO101LeaderConfig(port=port, id=arm_id)).calibration_fpath


def to_lerobot_dict(cal: dict[str, MotorCalibration]) -> dict:
    """转成 draccus 能 load 的纯 dict（dataclass 不能直接 json.dump）"""
    return {
        name: {
            "id": int(c.id),
            "drive_mode": int(c.drive_mode),
            "homing_offset": int(c.homing_offset),
            "range_min": int(c.range_min),
            "range_max": int(c.range_max),
        }
        for name, c in cal.items()
    }


def show_table(cal: dict[str, MotorCalibration]) -> None:
    print(f"  {'ID':<3} {'关节名':<15} {'Homing_Offset':>14} "
          f"{'Min':>7} {'Max':>7} {'跨度':>7} {'跨度(度)':>10}")
    print("  " + "-" * 70)
    for name, c in cal.items():
        span = c.range_max - c.range_min
        print(f"  {c.id:<3} {name:<15} {c.homing_offset:>14} "
              f"{c.range_min:>7} {c.range_max:>7} {span:>7} {span * DEG_PER_TICK:>9.1f}°")


def read_all(bus: FeetechMotorsBus) -> tuple[dict[str, MotorCalibration], dict]:
    """只读：校准值 + 一批状态寄存器"""
    cal = bus.read_calibration()
    extra: dict[str, dict] = {}
    for name in bus.motors:
        extra[name] = {}
        for reg in EXTRA_REGS:
            try:
                extra[name][reg] = int(bus.read(reg, name, normalize=False))
            except Exception as e:
                extra[name][reg] = f"<读取失败: {type(e).__name__}>"
    return cal, extra


def cmd_backup(args) -> int:
    print("=" * 96)
    print(f"  只读备份  ·  {args.arm}  ·  {args.port}  ·  id={args.id}")
    print("=" * 96)
    print("  [安全] 本模式只发 PING / READ，不写任何寄存器，机械臂不会运动\n")

    bus, is_raw = make_bus(args)
    bus.connect()
    try:
        cal, extra = read_all(bus)
        ping_err = bus.ping_errors() if is_raw else None
    finally:
        bus.disconnect(disable_torque=False)

    print("  从舵机 EEPROM 读到的校准值：\n")
    show_table(cal)

    # 电压窗口单独列出来 —— 这是"lerobot 认不认这台臂"的关键。
    # 判据用「实际电压是否落在窗口内」，不依赖错误标志：
    #   · 裸协议模式下能直接看到错误字节，最直观
    #   · 走 lerobot 的模式下握手能过就说明没有错误标志，此时显示「握手已通过」
    # 两种模式给出的结论一致，不会出现"没报错但其实有问题"的假阴性。
    print()
    print("  电压保护窗口 vs 实际电压：")
    print(f"  {'ID':<3} {'关节名':<15} {'允许窗口':>14} {'实际':>7} {'判定':>26}")
    print("  " + "-" * 70)
    trouble = []
    for name, m in so101_motors().items():
        d = extra[name]
        vmnv, vmxv = d.get("Min_Voltage_Limit"), d.get("Max_Voltage_Limit")
        pv = d.get("Present_Voltage")
        know = all(isinstance(x, int) for x in (vmnv, vmxv, pv))
        over = know and not (vmnv <= pv <= vmxv)
        if over:
            trouble.append((name, vmnv, vmxv, pv))
            verdict = "超窗 -> 应答带电压错误"
        elif ping_err and name in ping_err and ping_err[name][0] > 0:
            verdict = ping_err[name][1]
        elif know:
            verdict = "窗口内" + ("" if is_raw else "（握手已通过）")
        else:
            verdict = "读数缺失"
        window = f"{vmnv}~{vmxv}" if isinstance(vmnv, int) and isinstance(vmxv, int) else "?"
        print(f"  {m.id:<3} {name:<15} {window:>14} {str(pv):>7} {verdict:>26}")
    if trouble:
        print()
        print("  [!!] 这些关节的供电超出了它自己设定的窗口，应答包会带「输入电压错误」标志。")
        print("       lerobot 的握手会把带错误标志的舵机**整个过滤掉**，结果是「发现 0 个舵机」，")
        print("       lerobot-calibrate / lerobot-teleoperate 直接没法用 —— 但硬件本身是好的。")
        print("       修法：把电压窗口设回标准值（从臂就是标准值）：")
        for name, vmnv, vmxv, pv in trouble:
            print(f"         {name:<15} 现在 {vmnv}~{vmxv} -> "
                  f"改成 {DEFAULT_VOLTAGE_WINDOW[0]}~{DEFAULT_VOLTAGE_WINDOW[1]}")
        print(f"       命令：--voltage-window {DEFAULT_VOLTAGE_WINDOW[0]} "
              f"{DEFAULT_VOLTAGE_WINDOW[1]} --yes")

    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"{args.arm}_{args.id}_{ts}"

    raw_path = BACKUP_DIR / f"{stem}.json"
    raw_path.write_text(
        json.dumps(
            {
                "arm": args.arm,
                "id": args.id,
                "port": args.port,
                "backed_up_at": datetime.now().isoformat(timespec="seconds"),
                "note": "从舵机 EEPROM 只读读出。Homing_Offset=地址31, "
                        "Min_Position_Limit=地址9, Max_Position_Limit=地址11, "
                        "Min/Max_Voltage_Limit=地址15/14",
                "lerobot_calibration": to_lerobot_dict(cal),
                "voltage_window": {
                    name: {
                        "min": extra[name].get("Min_Voltage_Limit"),
                        "max": extra[name].get("Max_Voltage_Limit"),
                    }
                    for name in extra
                },
                "extra_registers": extra,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    # 另外存一份"干净"的 lerobot calibration 格式，可以原样拷到 calibration 目录
    lerobot_path = BACKUP_DIR / f"{args.arm}_{args.id}_lerobot.json"
    lerobot_path.write_text(
        json.dumps(to_lerobot_dict(cal), ensure_ascii=False, indent=4),
        encoding="utf-8",
    )

    # 再存一份稳定名字，方便 --restore 直接引用
    latest_path = BACKUP_DIR / f"{args.arm}_{args.id}.json"
    shutil.copyfile(raw_path, latest_path)

    print()
    print("  已保存：")
    print(f"    {raw_path}")
    print(f"      └ 完整记录（含备份时的位置/电压/温度，便于日后回看）")
    print(f"    {lerobot_path}")
    print(f"      └ 纯 lerobot 格式，可直接当 calibration 文件用")
    print(f"    {latest_path}")
    print(f"      └ 同样的内容，固定文件名，--restore 不带路径时默认用它")
    print()
    print("  下一步三选一：")
    print(f"    1) 装成 lerobot 的校准文件（推荐，不碰 EEPROM）：")
    print(f"         ... 05_backup_calibration.py --port {args.port} --arm {args.arm} "
          f"--id {args.id} --install")
    print(f"    2) 照常跑 lerobot-calibrate（会覆盖 EEPROM，但已有备份可回滚）")
    print(f"    3) 什么都不做，备份放在那儿")
    return 0


def cmd_install(args) -> int:
    """把 EEPROM 里读出来的值装成 lerobot 的 calibration 文件。不碰 EEPROM。"""
    lerobot_src = BACKUP_DIR / f"{args.arm}_{args.id}_lerobot.json"
    if not lerobot_src.exists():
        print(f"[!!] 找不到 {lerobot_src}")
        print(f"     先跑一次备份：--port {args.port} --arm {args.arm} --id {args.id}")
        return 1

    dst = calibration_target(args.arm, args.id, args.port)
    payload = json.loads(lerobot_src.read_text(encoding="utf-8"))

    print("=" * 96)
    print("  安装 lerobot 校准文件（只写文件，不碰舵机）")
    print("=" * 96)
    print(f"  来源  {lerobot_src}")
    print(f"  目标  {dst}")
    print()

    # 让工具校验一遍：文件值 vs 舵机 EEPROM 值 是否一致
    bus, _ = make_bus(args)
    bus.connect()
    try:
        eeprom = bus.read_calibration()
    finally:
        bus.disconnect(disable_torque=False)

    eep = to_lerobot_dict(eeprom)
    mismatch = []
    for name, want in payload.items():
        got = eep.get(name)
        if got is None:
            mismatch.append((name, "舵机上没这个关节"))
            continue
        for k in ("homing_offset", "range_min", "range_max"):
            if int(want[k]) != int(got[k]):
                mismatch.append((name, f"{k}: 文件 {want[k]} vs 舵机 {got[k]}"))

    if mismatch:
        print("  [!!] 文件里的值和现在舵机 EEPROM 里的值不一致：")
        for name, why in mismatch:
            print(f"       {name:<15} {why}")
        print()
        print("  → 说明 EEPROM 在备份之后被改过。先重新备份一遍再装。")
        return 1

    print("  [OK] 文件值与舵机 EEPROM 完全一致")
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        bak = dst.with_suffix(dst.suffix + f".bak_{datetime.now():%Y%m%d_%H%M%S}")
        shutil.copyfile(dst, bak)
        print(f"  [!]  目标已存在，旧文件先备份到 {bak.name}")
    shutil.copyfile(lerobot_src, dst)
    print(f"  已写入 {dst}")
    print()
    print("  现在 lerobot 启动时 bus.is_calibrated 会返回 True ——")
    print("  也就是说，不用跑 lerobot-calibrate 就能直接遥操作，")
    print("  前任的校准原封不动保留在舵机里。")
    return 0


def cmd_restore(args) -> int:
    """把备份写回 EEPROM。会写舵机，必须 --yes。"""
    src = Path(args.restore)
    if not src.exists():
        print(f"[!!] 找不到备份文件 {src}")
        return 1

    data = json.loads(src.read_text(encoding="utf-8"))
    want: dict = data.get("lerobot_calibration", data)

    print("=" * 96)
    print("  从备份恢复 EEPROM（会写舵机）")
    print("=" * 96)
    print(f"  备份文件  {src}")
    if "backed_up_at" in data:
        print(f"  备份时间  {data['backed_up_at']}")
        print(f"  备份自    {data.get('arm')} / {data.get('port')}")
    print()

    if not args.yes:
        print("  将要写入的值：\n")
        for name, c in want.items():
            print(f"    {c['id']:<3} {name:<15} Homing_Offset={c['homing_offset']:>6}  "
                  f"Min={c['range_min']:>5}  Max={c['range_max']:>5}")
        print()
        print("  [dry-run] 没有写任何东西。确认无误后加 --yes 真正执行。")
        return 0

    print("  [!] 注意：本脚本不主动开关扭矩。")
    print("      如果你现在没用手托着臂 / 臂处在高位，先确认掉电不会砸下来。")
    print()

    bus, _ = make_bus(args)
    bus.connect()
    try:
        before = to_lerobot_dict(bus.read_calibration())

        print("  写入中（EEPROM 需先解锁 Lock=0，写完再锁回 Lock=1）...\n")
        ok, fail = [], []
        for name, c in want.items():
            if name not in bus.motors:
                fail.append((name, "这个臂上没有该关节"))
                continue
            try:
                bus.write("Lock", name, 0)
                bus.write("Homing_Offset", name, int(c["homing_offset"]))
                bus.write("Min_Position_Limit", name, int(c["range_min"]))
                bus.write("Max_Position_Limit", name, int(c["range_max"]))
                bus.write("Lock", name, 1)
                ok.append(name)
            except Exception as e:
                fail.append((name, f"{type(e).__name__}: {e}"))
                try:
                    bus.write("Lock", name, 1)  # 无论如何把锁锁回去
                except Exception:
                    pass

        time.sleep(0.2)
        after = to_lerobot_dict(bus.read_calibration())
    finally:
        bus.disconnect(disable_torque=False)

    print("  结果：\n")
    print(f"  {'ID':<3} {'关节名':<15} {'改动':<10} {'Homing_Offset':>22} "
          f"{'Min':>15} {'Max':>15} {'回读校验':>10}")
    print("  " + "-" * 100)
    bad = 0
    for name, c in want.items():
        b, a = before.get(name), after.get(name)
        if a is None:
            print(f"  {c['id']:<3} {name:<15} {'—':<10} {'读不回来':>22}")
            bad += 1
            continue
        changed = b != a
        mark = "已改" if changed else "本来就对"
        cells = []
        for k in ("homing_offset", "range_min", "range_max"):
            bs, as_ = (b or {}).get(k), a.get(k)
            if bs == as_ == c[k]:
                cells.append(f"{as_}")
            else:
                cells.append(f"{bs}->{as_}(要{c[k]})")
        verify = "OK" if a == {
            "id": c["id"], "drive_mode": c["drive_mode"],
            "homing_offset": c["homing_offset"],
            "range_min": c["range_min"], "range_max": c["range_max"],
        } else "不一致"
        if verify != "OK":
            bad += 1
        print(f"  {c['id']:<3} {name:<15} {mark:<10} {cells[0]:>22} "
              f"{cells[1]:>15} {cells[2]:>15} {verify:>10}")

    if fail:
        print()
        for name, why in fail:
            print(f"  [!!] {name} 写入失败：{why}")

    print()
    if bad == 0 and not fail:
        print("  [OK] 全部写回并回读校验通过。EEPROM 已恢复到备份时的状态。")
        return 0
    print(f"  [!!] 有 {bad + len(fail)} 个关节没到位，检查上面的表格。")
    return 1


def _write_locked(bus, name: str, reg: str, value: int) -> None:
    """EEPROM 写入的标准三步：解锁 -> 写 -> 锁回。

    不解锁也能写，但舵机可能静默忽略；锁回去是防止后续误写。
    """
    bus.write("Lock", name, 0)
    try:
        bus.write(reg, name, int(value))
    finally:
        bus.write("Lock", name, 1)


def cmd_voltage_window(args) -> int:
    """设置舵机自己的电压保护窗口（EEPROM 地址 14/15）。

    这是主臂用不了 lerobot 的根治手段：它的 Max_Voltage_Limit 被设成 12.0V，
    而供电 12.5V 稳压在其上，于是每个应答包都带「输入电压错误」，
    lerobot 的握手把 6 个舵机全过滤掉。

    改这个**不会**让舵机变得更脆弱：窗口只是"超出就报警"的阈值，
    真正的过压保护在电源侧。标准 STS3215 窗口是 4.0~14.0V，从臂就是这个值。
    """
    lo, hi = args.voltage_window
    if not (0 < lo < hi <= 255):
        print(f"[!!] 窗口不合法：{lo}~{hi}（要求 0 < min < max <= 255，单位 0.1V）")
        return 1

    print("=" * 96)
    print(f"  设置电压保护窗口  ·  {args.port}  ·  目标 {lo}~{hi}  ({lo/10:.1f}V ~ {hi/10:.1f}V)")
    print("=" * 96)

    bus, _ = make_bus(args)
    bus.connect()
    try:
        before = {n: (bus.read("Min_Voltage_Limit", n), bus.read("Max_Voltage_Limit", n),
                      bus.read("Present_Voltage", n)) for n in bus.motors}

        print(f"\n  {'ID':<3} {'关节名':<15} {'当前窗口':>12} {'实际电压':>10} "
              f"{'是否超窗':>10}")
        print("  " + "-" * 60)
        need = []
        for name, m in bus.motors.items():
            vmn, vmx, pv = before[name]
            over = not (vmn <= pv <= vmx)
            if over or (vmn, vmx) != (lo, hi):
                need.append(name)
            print(f"  {m.id:<3} {name:<15} {f'{vmn}~{vmx}':>12} "
                  f"{f'{pv/10:.1f}V':>10} {'是 [!!]' if over else '否':>10}")

        if not args.yes:
            print()
            print(f"  [dry-run] 会把 {len(need)} 个关节的窗口设成 {lo}~{hi}：{need}")
            print("           没有写任何东西。确认后加 --yes 执行。")
            return 0

        print(f"\n  写入中（{len(need)} 个关节）...")
        fail = []
        for name in need:
            try:
                _write_locked(bus, name, "Min_Voltage_Limit", lo)
                _write_locked(bus, name, "Max_Voltage_Limit", hi)
            except Exception as e:
                fail.append((name, f"{type(e).__name__}: {e}"))

        time.sleep(0.3)
        after = {n: (bus.read("Min_Voltage_Limit", n), bus.read("Max_Voltage_Limit", n),
                     bus.read("Present_Voltage", n)) for n in bus.motors}

        print(f"\n  {'ID':<3} {'关节名':<15} {'改前':>10} {'改后':>10} {'电压':>8} {'校验':>8}")
        print("  " + "-" * 60)
        bad = 0
        for name, m in bus.motors.items():
            vmn0, vmx0, _ = before[name]
            vmn1, vmx1, pv = after[name]
            ok = (vmn1, vmx1) == (lo, hi)
            if not ok:
                bad += 1
            print(f"  {m.id:<3} {name:<15} {f'{vmn0}~{vmx0}':>10} "
                  f"{f'{vmn1}~{vmx1}':>10} {f'{pv/10:.1f}V':>8} {'OK' if ok else '失败':>8}")

        print()
        if fail:
            for name, why in fail:
                print(f"  [!!] {name} 写入失败：{why}")
        if bad == 0 and not fail:
            print("  [OK] 全部写入并回读校验通过。")
            print()
            print("  下一步：重新 PING 看错误标志还在不在 ——")
            print(f"    E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py "
                  f"--port {args.port} --arm {args.arm} --id {args.id}")
            print("  错误标志清掉后，lerobot 的握手就能过了。")
            return 0
        print(f"  [!!] 有 {bad + len(fail)} 个关节没改成。")
        return 1
    finally:
        bus.disconnect(disable_torque=False)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="SO-101 校准备份 / 安装 / 恢复（默认只读）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", required=True, help="串口，如 COM23（从臂）/ COM32（主臂）")
    ap.add_argument("--arm", required=True, choices=["follower", "leader"],
                    help="follower=从臂，leader=主臂")
    ap.add_argument("--id", required=True, help="lerobot 的 id，如 follower1 / leader1")
    ap.add_argument("--install", action="store_true",
                    help="把备份装成 lerobot 的 calibration 文件（不碰 EEPROM）")
    ap.add_argument("--restore", metavar="备份JSON",
                    help="从备份写回 EEPROM（会写舵机，需配 --yes）")
    ap.add_argument("--voltage-window", nargs=2, type=int, metavar=("MIN", "MAX"),
                    help="设置电压保护窗口（单位 0.1V，如 40 140 = 4.0~14.0V）。"
                         "会写舵机 EEPROM，需配 --yes。这是修「lerobot 发现 0 个舵机」用的")
    ap.add_argument("--raw", action="store_true",
                    help="强制走裸串口协议，不走 lerobot 的 bus（握手过不去时用）")
    ap.add_argument("--yes", action="store_true", help="真正执行写入，否则只预览")
    args = ap.parse_args()

    chosen = [x for x in (args.install, args.restore, args.voltage_window) if x]
    if len(chosen) > 1:
        ap.error("--install / --restore / --voltage-window 只能用一个")
    if args.install:
        return cmd_install(args)
    if args.restore:
        return cmd_restore(args)
    if args.voltage_window:
        return cmd_voltage_window(args)
    return cmd_backup(args)


if __name__ == "__main__":
    sys.exit(main())
