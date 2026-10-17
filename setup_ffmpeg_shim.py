#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
让 torchcodec 找到 FFmpeg —— 复用 PyAV 自带的 FFmpeg DLL

【问题】
  lerobot 录制数据集依赖 torchcodec。torchcodec 自己的 libtorchcodec_core*.dll
  是**动态链接 FFmpeg** 的。实测导入表：

      libtorchcodec_core7.dll -> avcodec-61.dll / avformat-61.dll / avutil-59.dll
                                 swscale-8.dll / swresample-5.dll / avfilter-10.dll
                                 （外加 c10.dll / torch_cpu.dll，这两个由 torch 提供）

  FFmpeg 主版本映射：core8->FFmpeg8, core7->FFmpeg7.1, core6->FFmpeg7.0。
  加载循环是 `for ffmpeg_major_version in (8, 7, 6, 5, 4)`。

【为什么这台机器上装不上 FFmpeg】
  1. github.com 不可达，winget 下载 FFmpeg 直接
     `InternetOpenUrl() failed. 0x80072efd`
  2. winget 提供的是 FFmpeg 9.0.2，而 torchcodec 0.11.1 的循环是 (8,7,6,5,4)，
     根本不认版本 9

【解决办法：复用 PyAV 的 FFmpeg】
  lerobot 自己会装 PyAV（av>=15,<16），而 PyAV 15.x 的轮子**自带 FFmpeg 7.1
  的动态库**，就在 site-packages/av.libs/ 里。

  但有两处对不上，缺一不可：

  (1) 文件名带哈希后缀
        av.libs/avcodec-61-1306a7df8fab62262dd7797a8fb9f1cc.dll
      torchcodec 找的是不带后缀的 avcodec-61.dll -> 找不到。
      => 复制一份去掉后缀的，放进 _ffmpeg_shim/ 和 torchcodec/

  (2) av.libs 不在 DLL 搜索路径上
      avcodec-61.dll 自己还要依赖 24 个 DLL（libx264 / libx265 / libvpx /
      libgcc_s_seh-1 / libstdc++-6 / libwinpthread-1 ...），这些也在 av.libs 里，
      而且**同样带哈希后缀**。
      => 必须把 av.libs 整个目录加进 DLL 搜索路径

  注意 (2) 是**充分必要**的：实测只加 _ffmpeg_shim 会失败，加上 av.libs 才成功。

【实测有效的三种做法】（都验证过）
  A. 先 `import av` 再 `import torchcodec`（PyAV 自己会注册 av.libs）
  B. `os.add_dll_directory(site-packages/av.libs)`
  C. 本脚本：写一个 .pth 引导文件，让每个用到这个 venv 的 Python 进程
     启动时自动注册 —— 这样调用方什么都不用改

  本脚本用 C，并把 A 作为兜底写进 README。

【为什么不能直接改 av.libs 里的文件名】
  PyAV 内部是用哈希名互相引用的，重命名会直接搞坏 PyAV。

【用法】
  python setup_ffmpeg_shim.py            # 应用
  python setup_ffmpeg_shim.py --check    # 只检查（含子进程实测）
  python setup_ffmpeg_shim.py --undo     # 撤销
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import site
import subprocess
import sys
import sysconfig
from pathlib import Path

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

# avcodec-61-1306a7df8fab62262dd7797a8fb9f1cc.dll  ->  avcodec-61.dll
HASHED = re.compile(r"^((?:av|sw|postproc)[a-z]+-\d+)-[0-9a-f]{16,}\.dll$", re.I)
# torchcodec 目录里我们自己放进去的 FFmpeg DLL（undo 时按这个模式清理）
CANONICAL = re.compile(r"^(?:av|sw|postproc)[a-z]+-\d+\.dll$", re.I)

PTH_NAME = "_ffmpeg_shim.pth"


def site_packages() -> Path:
    """
    可靠地定位 site-packages。

    不能用 site.getsitepackages()[0]：Anaconda 建的 venv 里它返回的是
    ['E:\\lerobot-env', 'E:\\lerobot-env\\Lib\\site-packages']，
    第一项是 venv 根目录而不是 site-packages。.pth 文件放在 venv 根目录
    是不会被 Python 处理的，会导致整个 shim 静默失效。
    """
    purelib = Path(sysconfig.get_paths()["purelib"])
    if purelib.is_dir() and purelib.name == "site-packages":
        return purelib
    for cand in site.getsitepackages():
        if (Path(cand) / "lerobot").is_dir():
            return Path(cand)
    return purelib


def find_pyav_libs(sp: Path) -> Path | None:
    """定位 PyAV 放 FFmpeg DLL 的目录（av.libs）。"""
    for cand in (sp / "av.libs", sp / "av" / ".libs"):
        if cand.is_dir() and any(cand.glob("avcodec-*.dll")):
            return cand
    for p in sp.rglob("avcodec-*.dll"):
        return p.parent
    return None


def collect_hashed(libs: Path) -> dict[str, Path]:
    """{规范名: 原始（哈希名）文件路径}"""
    out = {}
    for f in sorted(libs.glob("*.dll")):
        m = HASHED.match(f.name)
        if m:
            out[m.group(1) + ".dll"] = f
    return out


def which_ffmpeg_major(names) -> int | None:
    """从 avcodec-NN.dll 推断 FFmpeg 主版本。"""
    mapping = {62: 8, 61: 7, 60: 7, 59: 6, 58: 5, 57: 4}
    for n in names:
        m = re.match(r"avcodec-(\d+)\.dll", n)
        if m:
            return mapping.get(int(m.group(1)))
    return None


def pth_line(libs: Path | None, shim: Path) -> str:
    """
    生成 .pth 内容。.pth 每行必须是单行可 exec 的语句。

    av.libs 是**必需**的（见模块开头说明），shim 目录提供去掉哈希后缀的规范名。
    只加存在的目录，避免 PyAV 没装时报错。
    """
    dirs = [str(d) for d in (libs, shim) if d is not None]
    inner = ", ".join(f"r'{d}'" for d in dirs)
    return (
        "import os; "
        f"[os.add_dll_directory(p) for p in ({inner},) if os.path.isdir(p)]\n"
    )


def verify_fresh() -> bool:
    """
    在**全新子进程**里验证。

    必须这么做：.pth 只在解释器启动时被处理，
    在当前进程里 import torchcodec 测的是"还没生效"的状态，结果不可信。
    """
    code = (
        "import warnings; warnings.filterwarnings('ignore')\n"
        "import torchcodec\n"
        "from torchcodec._core import ops\n"
        "print('FFMPEG_MAJOR=' + str(ops.ffmpeg_major_version))\n"
    )
    try:
        r = subprocess.run(
            [sys.executable, "-c", code],
            capture_output=True, text=True, timeout=300,
        )
    except subprocess.TimeoutExpired:
        print("    [!!] 子进程超时（300s）")
        return False
    if r.returncode == 0 and "FFMPEG_MAJOR=" in r.stdout:
        major = r.stdout.split("FFMPEG_MAJOR=")[1].split()[0]
        print(f"    [OK] 全新子进程里 import torchcodec 成功，"
              f"自动选中 FFmpeg 主版本 {major}")
        return True
    tail = (r.stderr or r.stdout or "").strip().splitlines()
    print("    [!!] 全新子进程里仍然失败：")
    for line in tail[-6:]:
        print(f"         {line}")
    return False


def check() -> int:
    sp = site_packages()
    libs = find_pyav_libs(sp)
    shim = sp / "_ffmpeg_shim"
    pth = sp / PTH_NAME
    tc = sp / "torchcodec"

    print("=" * 72)
    print("  FFmpeg shim 状态检查")
    print("=" * 72)
    print(f"  site-packages : {sp}")
    print(f"  PyAV DLL 目录 : {libs or '未找到（PyAV 还没装？）'}")
    print(f"  shim 目录     : {shim}  {'存在' if shim.is_dir() else '不存在'}")
    print(f"  .pth 引导文件 : {pth}  {'存在' if pth.is_file() else '不存在'}")

    if libs:
        found = collect_hashed(libs)
        n = which_ffmpeg_major(found)
        print(f"  可用 FFmpeg   : 主版本 {n}（torchcodec 支持 4~8）")
        print(f"  可去后缀的 DLL: {len(found)} 个 -> {sorted(found)[:4]}...")
        print(f"  av.libs 总 DLL: {len(list(libs.glob('*.dll')))} 个"
              f"（avcodec 的传递依赖都在这里）")

    if shim.is_dir():
        print(f"  shim 内 DLL   : {len(list(shim.glob('*.dll')))} 个")
    if tc.is_dir():
        n_canon = len([p for p in tc.glob("*.dll") if CANONICAL.match(p.name)])
        print(f"  torchcodec 内 : {n_canon} 个规范名 FFmpeg DLL")

    # .pth 是否覆盖了 av.libs —— 这是最容易漏掉的一环
    if pth.is_file():
        body = pth.read_text(encoding="utf-8", errors="replace")
        ok_libs = libs is not None and str(libs) in body
        print(f"  .pth 含 av.libs: {'是' if ok_libs else '否  <-- 会失败，需要重新运行 apply'}")
        if not ok_libs:
            print(f"    当前内容: {body.strip()}")

    print()
    print("  全新子进程实测 ……")
    ok = verify_fresh()
    return 0 if ok else 1


def apply(force: bool = False) -> int:
    sp = site_packages()
    libs = find_pyav_libs(sp)
    if libs is None:
        print("[!!] 找不到 PyAV 的 FFmpeg DLL。先装 av：")
        print("     pip install 'av>=15,<16'")
        return 1

    found = collect_hashed(libs)
    if not found:
        print(f"[!!] {libs} 里没有找到带哈希后缀的 FFmpeg DLL")
        return 1

    major = which_ffmpeg_major(found)
    if major is None or not (4 <= major <= 8):
        print(f"[!!] FFmpeg 主版本 {major} 不在 torchcodec 支持的 4~8 范围内")
        print("     （FFmpeg 9 就是这种情况，torchcodec 不认）")
        return 1

    shim = sp / "_ffmpeg_shim"
    shim.mkdir(exist_ok=True)
    print(f"  来源   : {libs}")
    print(f"  shim   : {shim}")
    print(f"  FFmpeg : 主版本 {major}（torchcodec 支持 4~8，可用）")
    print()

    n = 0
    for canonical, src in sorted(found.items()):
        dst = shim / canonical
        if dst.exists() and not force:
            continue
        shutil.copy2(src, dst)
        print(f"    {src.name}")
        print(f"      -> {canonical}")
        n += 1

    # 同时在 torchcodec 自己目录里放一份规范名。
    # LoadLibrary 加载 core7.dll 时会把它所在目录作为"被加载 DLL 的目录"搜索，
    # 命中率最高（实测只靠这一条 + av.libs 就够）。
    tc = sp / "torchcodec"
    if tc.is_dir():
        print()
        print(f"  同步规范名到 torchcodec 目录: {tc}")
        for canonical in sorted(found):
            dst = tc / canonical
            if dst.exists() and not force:
                continue
            shutil.copy2(shim / canonical, dst)
            print(f"    {canonical}")

    # .pth 引导：让每个用这个 venv 的 Python 进程自动注册 DLL 搜索路径
    pth = sp / PTH_NAME
    line = pth_line(libs, shim)
    if not pth.exists() or pth.read_text(encoding="utf-8", errors="replace") != line:
        pth.write_text(line, encoding="utf-8")
        print()
        print(f"  写入引导文件: {pth}")
        print(f"    {line.strip()}")

    print()
    print(f"  完成，新增 {n} 个规范名副本。")
    print()
    return check()


def undo() -> int:
    sp = site_packages()
    shim = sp / "_ffmpeg_shim"
    pth = sp / PTH_NAME
    tc = sp / "torchcodec"

    if pth.is_file():
        pth.unlink()
        print(f"  已删除 {pth}")
    if shim.is_dir():
        shutil.rmtree(shim)
        print(f"  已删除 {shim}")
    if tc.is_dir():
        removed = 0
        for dll in tc.glob("*.dll"):
            if CANONICAL.match(dll.name):
                dll.unlink()
                removed += 1
        if removed:
            print(f"  已从 torchcodec 目录移除 {removed} 个 FFmpeg DLL")
    print("  撤销完成（av.libs 原文件未动）。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="让 torchcodec 复用 PyAV 自带的 FFmpeg DLL")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--check", action="store_true", help="只检查状态（含子进程实测）")
    g.add_argument("--undo", action="store_true", help="撤销")
    ap.add_argument("--force", action="store_true", help="覆盖已存在的文件")
    args = ap.parse_args()

    if args.check:
        return check()
    if args.undo:
        return undo()
    return apply(force=args.force)


if __name__ == "__main__":
    sys.exit(main())
