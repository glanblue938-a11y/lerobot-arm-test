#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线自检：不需要真机械臂，用假的串口对象验证协议组包/解包逻辑是否正确。
这是给我（Claude）自己验证脚本用的，你不需要跑它。
    python _selftest.py
"""
import sys
import types

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(errors="replace")
    except Exception:
        pass

# --- 用一个假的 serial 模块顶替 pyserial，这样没装 pyserial 也能自检 ---
fake = types.ModuleType("serial")
fake.SerialException = type("SerialException", (Exception,), {})
fake.Serial = object
tools = types.ModuleType("serial.tools")
listports = types.ModuleType("serial.tools.list_ports")
listports.comports = lambda: []
tools.list_ports = listports
fake.tools = tools
sys.modules["serial"] = fake
sys.modules["serial.tools"] = tools
sys.modules["serial.tools.list_ports"] = listports

import importlib

scan = importlib.import_module("01_scan_bus")
joint = importlib.import_module("02_joint_test")

failures = []


def check(label, got, want):
    if got == want:
        print(f"  ok   {label}")
    else:
        print(f"  FAIL {label}: got {got!r}, want {want!r}")
        failures.append(label)


print("== 1. 校验和 / 组包 ==")
# 标准 PING 包：FF FF 01 02 01 FB
check("PING id=1", scan.build_instruction(1, scan.INST_PING), b"\xff\xff\x01\x02\x01\xfb")
check("PING 两个模块一致", joint.build_packet(1, joint.INST_PING), b"\xff\xff\x01\x02\x01\xfb")
# 校验和算法：sum & 取反
check("checksum([1,2,1])", scan.checksum(bytes([1, 2, 1])), 0xFB)
check("checksum([0xFF,0xFF,0xFF])", scan.checksum(bytes([0xFF, 0xFF, 0xFF])), 0x02)
# 校验和的定义性质：原数据 + 校验和 必须补满 0xFF
for _probe in (bytes([1, 2, 1]), bytes([0xFF, 0xFF, 0xFF]), bytes([3, 4, 2, 56, 2]), bytes([0])):
    check(f"checksum 自洽性 {list(_probe)}",
          (sum(_probe) + scan.checksum(_probe)) & 0xFF, 0xFF)

print("\n== 2. READ 包（读位置: 地址56, 长度2）==")
pkt = scan.build_instruction(3, scan.INST_READ, bytes([56, 2]))
check("READ 包头", pkt[:5], b"\xff\xff\x03\x04\x02")
check("READ 参数", pkt[5:7], bytes([56, 2]))
check("READ 校验和", pkt[7], (~(3 + 4 + 2 + 56 + 2)) & 0xFF)


class FakeSerial:
    """把预设的字节流当作舵机的应答吐出来。"""

    def __init__(self, data: bytes = b""):
        self.data = bytearray(data)

    def read(self, n=1):
        out = bytes(self.data[:n])
        del self.data[:n]
        return out

    def write(self, payload):
        return len(payload)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class DeadBusSerial(FakeSerial):
    """模拟一个真实存在、但总线上什么都没有的串口。"""

    def __init__(self, port, baud, timeout=0.02):
        super().__init__(b"")


dead = types.ModuleType("serial")
dead.SerialException = fake.SerialException
dead.Serial = DeadBusSerial
dead.tools = tools


print("\n== 3. 状态包解析 ==")
# 正常应答: FF FF 01 04 00 00 08 XX  -> id=1, len=4, err=0, pos=0x0800=2048
body = bytes([1, 4, 0, 0x00, 0x08])
good = b"\xff\xff" + body + bytes([scan.checksum(body)])
res = scan.read_status(FakeSerial(good))
check("id", res and res["id"], 1)
check("error", res and res["error"], 0)
check("position bytes", res and res["params"], b"\x00\x08")
check("位置解码 2048", int.from_bytes(res["params"][:2], "little"), 2048)

print("\n== 4. 坏包必须被拒绝 ==")
check("校验和错的包", scan.read_status(FakeSerial(good[:-1] + b"\x00")), None)
check("空数据", scan.read_status(FakeSerial(b"")), None)
check("半截包", scan.read_status(FakeSerial(b"\xff\xff\x01\x04\x00")), None)

print("\n== 5. 前导噪声不应干扰同步 ==")
noisy = b"\x00\x13\xaa" + good
res = scan.read_status(FakeSerial(noisy))
check("跳过噪声后仍能解析", res and res["id"], 1)

print("\n== 6. 数值解释 ==")
check("负载 bit10 符号位 +100", scan.decode_load(100), 100)
check("负载 bit10 符号位 -100", scan.decode_load(0x400 | 100), -100)
check("负载 0", scan.decode_load(0), 0)
check("负载 满量程 1023", scan.decode_load(1023), 1023)
check("角度换算 4096 tick = 360°", round(scan.ticks_to_deg(4096), 6), 360.0)
check("角度换算 1 tick ≈ 0.0879°", round(scan.ticks_to_deg(1), 4), 0.0879)

print("\n== 7. 关节名映射 ==")
check("SO-101 六关节", sorted(scan.JOINT_NAMES), [1, 2, 3, 4, 5, 6])
check("ID6 是夹爪", scan.JOINT_NAMES[6], "gripper")

print("\n== 8. 安全闸门（不会真的碰硬件，因为都在这之前就返回了）==")


def run_main(mod, argv):
    """跑一次 main()，返回 (退出码, 是否抛了 SystemExit)。"""
    old = sys.argv
    sys.argv = ["test"] + argv
    try:
        return mod.main(), False
    except SystemExit as e:
        return (e.code if isinstance(e.code, int) else 1), True
    finally:
        sys.argv = old


# --- 01_scan_bus：只读工具，--list 必须能正常返回 ---
code, _ = run_main(scan, ["--list"])
check("01 --list 正常退出", code, 0)

# --- 02_joint_test：会驱动电机的脚本，闸门必须拦得住 ---
code, _ = run_main(joint, ["--port", "COM99", "--id", "1"])
check("02 不给 --yes 时拒绝执行(码2)", code, 2)

# 参数校验发生在开串口之前
code, exited = run_main(joint, ["--port", "COM99", "--id", "1", "--delta", "90", "--yes"])
check("02 幅度 90 度被拒绝", exited, True)

code, exited = run_main(joint, ["--port", "COM99", "--id", "9", "--yes"])
check("02 非法 ID 被拒绝", exited, True)

code, exited = run_main(joint, ["--port", "COM99", "--id", "abc", "--yes"])
check("02 非数字 ID 被拒绝", exited, True)

# 换成"空总线"的假串口，验证拿到 --yes 之后能正常走完流程而不是崩掉
sys.modules["serial"] = dead
scan.serial = dead
joint.serial = dead

code, _ = run_main(joint, ["--port", "COM99", "--id", "1", "--dry-run"])
check("02 --dry-run 不受 --yes 限制(码!=2)", code != 2, True)

code, _ = run_main(scan, ["--port", "COM99"])
check("01 空总线上不崩溃，返回 1", code, 1)

print()
if failures:
    print(f"[!!] {len(failures)} 项失败: {failures}")
    sys.exit(1)
print("[OK] 全部通过")
