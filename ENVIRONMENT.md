# 运行环境（本机实测记录）

> 本文档记录的是**这台机器上实际验证过的**事实，不是通用教程。
> 记录时间：2026-10-17

---

## 1. 硬件

| 项目 | 实测值 |
|---|---|
| 机械臂 | SO-101 主从臂，6 个 Feetech STS 系列总线舵机 |
| 总线适配器 | `USB-Enhanced-SERIAL CH343`，VID:PID = `1A86:55D3` |
| 串口 | **COM23** |
| 波特率 | **1000000**（出厂默认，未被改过） |
| 舵机 ID | 1~6，完全符合出厂配置 |
| 通信质量 | 连读 6/6 成功，位置抖动 0 tick（非常好） |
| 温度 | 31~34°C（空载） |
| 供电电压 | ✅ **12.4~12.5V，正常** |
| 舵机出厂电压窗口 | **4.0 ~ 14.0V**（EEPROM 地址 14/15，6 个舵机一致） |
| 自报故障标志 | 全部为 0（无电压/过流/过温/堵转） |

### 供电结论

**已解决**。早期读数是 5.1~5.2V（偏低），现在是 12.4~12.5V，
稳稳落在舵机自己的出厂窗口 4.0~14.0V 内。6 个舵机的故障标志全为 0。

判断依据不是 STS3215 的通用标称值，而是**直接读舵机 EEPROM 里的
`Min_Voltage_Limit`(地址 15) / `Max_Voltage_Limit`(地址 14)** ——
这是出厂设定、且每台一致，比任何文档都权威。

### 关于"电压读数是常数"

早期我怀疑过"读数恒定 = 寄存器地址读错了"。**这个推断是错的**，三重证据：

1. **地址 62 已被 lerobot 官方表确认**：
   `lerobot/motors/feetech/tables.py` 里就是 `"Present_Voltage": (62, 1)`
2. 同一批采样里**温度会跳**（32→40°C），说明 ADC 和读取链路是活的
3. 换电源后读数跟着变（5.2V → 12.5V）——配置寄存器不会自己变值

`03_power_check.py` 已按这个逻辑改过：只有在**电压和温度都 bit 级不变**时
才报警，否则"恒定"只说明电源稳。

```bash
E:/lerobot-env/Scripts/python.exe 03_power_check.py --port COM23
```

---

## 2. 显卡

| 项目 | 值 |
|---|---|
| GPU | NVIDIA GeForce RTX 4070（12GB 显存，Ada 架构 sm_89） |
| 驱动 | 580.88 |
| 驱动支持 CUDA | 13.0 |
| 选的 torch 构建 | **cu128**（CUDA 12.8，向下兼容新驱动） |

12GB 显存足够：ACT / Diffusion Policy 轻松，SmolVLA 微调要配合 LoRA。

---

## 3. Python 环境

| 项目 | 值 |
|---|---|
| 位置 | `E:\lerobot-env`（独立 venv，不污染 Anaconda） |
| 解释器 | `E:\lerobot-env\Scripts\python.exe` |
| Python | 3.12.7（lerobot 0.6.1 要求 ≥3.12） |
| 创建自 | `D:\Anaconda\python.exe` |

> **注意**：所有命令都要用 `E:/lerobot-env/Scripts/python.exe`，
> 直接敲 `python` 用的是 Anaconda 的，里面没装 lerobot。

### 为什么不用 conda

`conda` 在这个 shell 里不在 PATH 上，而且 venv 更轻、隔离更干净，
装在 E: 盘（1.2T 可用）也避开了 C:/D: 空间紧张的问题。

---

## 4. 软件包

| 包 | 版本 | 说明 |
|---|---|---|
| lerobot | 0.6.1 | 最新版，Python ≥3.12 |
| torch | 2.11.0+cu128 | lerobot 要求 ≥2.7,<2.12，这是上限内最新 |
| torchvision | 0.26.0+cu128 | 与 torch 2.11 配套 |
| pyserial | 3.5 | 诊断脚本用 |

### 国内网络下的源（重点）

**PyPI 直连会超时**，必须走镜像：

- 普通包 → 清华：`https://pypi.tuna.tsinghua.edu.cn/simple`
  （已固化在 `E:\lerobot-env\pip.ini`，以后不用再手动加 `-i`）
- **torch 的 CUDA 版 → 上交大**：`https://mirror.sjtu.edu.cn/pytorch-wheels/cu128`

实测速度对比（同一个 torch 轮子）：

| 源 | 速度 |
|---|---|
| download.pytorch.org（官方） | 0.3~0.7 MB/s |
| **mirror.sjtu.edu.cn** | **~2 MB/s** |

> ⚠️ `pip.ini` 里**不能写中文注释**。pip 用系统 locale（cp936/GBK）读这个文件，
> UTF-8 的中文字节会让它直接报
> `Configuration file contains invalid cp936 characters` 而拒绝启动。

### 关于 Aliyun 的 pytorch 镜像

`https://mirrors.aliyun.com/pytorch-wheels/cu128/` 响应很快，但它只是个**网页入口**，
不是 pip 能用的 PEP503 索引（`/torch/` 子路径返回 404）。用不了。
上交大那个才是标准索引（带 `pypi:repository-version` 标记）。

---

## 5. FFmpeg 问题（Windows 上的一个坑）

lerobot 录制数据集依赖 **torchcodec**，而 torchcodec 是**动态链接 FFmpeg 的**。
解析它的导入表可以确认：

| torchcodec 的 DLL | 导入的 FFmpeg | 对应 FFmpeg 版本 |
|---|---|---|
| `libtorchcodec_core8.dll` | `avcodec-62.dll` | 8.x |
| `libtorchcodec_core7.dll` | `avcodec-61.dll` | 7.1 |
| `libtorchcodec_core6.dll` | `avcodec-60.dll` | 7.0 |

加载顺序是 `for ffmpeg_major_version in (8, 7, 6, 5, 4)` —— 从高往低试，
哪个 FFmpeg 的 DLL 齐全就用哪个。

### 三个坑

1. **必须装 shared 版 FFmpeg**（带 DLL 的），静态版不行
2. **FFmpeg 9 不被支持**。winget 默认装的就是 9.0.2，装了也用不了
3. **github.com 在本地不可达**，winget 下载 FFmpeg 会失败
   （`InternetOpenUrl() failed. 0x80072efd`）

### 采用的方案：复用 PyAV 自带的 FFmpeg

lerobot 自己会装 PyAV（`av>=15,<16`），而 **PyAV 15.x 的轮子里自带 FFmpeg 7.1 的 DLL**，
就在 `site-packages/av.libs/`（34 个 DLL，81MB），版本正好对上 torchcodec 的
`core7`（需要 `avcodec-61`）。

**有两个缺口，缺一不可：**

#### 缺口 1：文件名带哈希后缀

```
av.libs/avcodec-61-1306a7df8fab62262dd7797a8fb9f1cc.dll
```

torchcodec 找的是不带后缀的 `avcodec-61.dll`，对不上。
→ 复制一份去掉后缀的，放进 `_ffmpeg_shim/` 和 `torchcodec/`。

#### 缺口 2（关键）：`av.libs` 不在 DLL 搜索路径上

`avcodec-61.dll` 自己还要依赖 **24 个 DLL**（`libx264` / `libx265` / `libvpx` /
`libaom` / `libdav1d` / `libopus` / `libgcc_s_seh-1` / `libstdc++-6` /
`libwinpthread-1` …，是 MinGW/GCC 编译的），这些**也在 av.libs 里、也带哈希后缀**。

→ 必须把 `av.libs` **整个目录**加进 DLL 搜索路径。

只做缺口 1 会失败。实测矩阵：

| 做法 | 结果 |
|---|---|
| 什么都不做 | ❌ |
| 只加 `_ffmpeg_shim` 到搜索路径 | ❌ |
| **加 `av.libs` 到搜索路径** | ✅ |
| 先 `import av` 再 `import torchcodec` | ✅ |
| 只把目录塞进 `PATH`（不加 av.libs） | ❌ |

#### 最终做法

`setup_ffmpeg_shim.py` 写一个 `.pth` 引导文件，让每个用这个 venv 的
Python 进程**启动时自动**注册 `av.libs` + `_ffmpeg_shim`：

```bash
E:/lerobot-env/Scripts/python.exe setup_ffmpeg_shim.py          # 应用
E:/lerobot-env/Scripts/python.exe setup_ffmpeg_shim.py --check  # 检查（含子进程实测）
E:/lerobot-env/Scripts/python.exe setup_ffmpeg_shim.py --undo   # 撤销
```

生成的 `site-packages/_ffmpeg_shim.pth`：

```python
import os; [os.add_dll_directory(p) for p in (r'...\av.libs', r'...\_ffmpeg_shim',) if os.path.isdir(p)]
```

> ⚠️ `.pth` 只在**解释器启动时**处理。所以 `--check` 必须开**子进程**验证 ——
> 在同一个进程里 `import torchcodec` 测的是"还没生效"的状态，结果不可信。
> 早期就是因为这个，明明 shim 装好了却一直报失败。

**兜底**（万一 `.pth` 被 `-S` / `-I` 跳过）：在 `import torchcodec` 之前
先 `import av` 即可，PyAV 自己会注册 `av.libs`。

**好处**：零额外下载、不需要管理员权限、不依赖 github。

### 验证结果

```bash
E:/lerobot-env/Scripts/python.exe setup_ffmpeg_shim.py --check
#   [OK] 全新子进程里 import torchcodec 成功，自动选中 FFmpeg 主版本 7
```

真编码/解码往返也测过（12 帧 → 30133 字节 → 解回 6 帧，元数据 12 帧），
libx264 正常工作。

---

## 6. 软限位：这副臂被前任**校准过**（重要）

首次跑 `02_joint_test.py --id all --yes` 时报了 ID 3、ID 4 两个"异常"。
**两个都不是故障**，是 EEPROM 里的软限位在正常工作。

### 实测限位值（EEPROM 地址 9 / 11）

| ID | 关节 | Min_Position_Limit | Max_Position_Limit | 跨度 | 当时位置 |
|---|---|---|---|---|---|
| 1 | shoulder_pan | 840 | 3258 | 212.5° | 2288 |
| 2 | shoulder_lift | **869** | 3098 | 195.9° | 873（贴下限，差 4） |
| 3 | elbow_flex | 983 | **3188** | 193.8° | **3185（贴上限，差 3）** |
| 4 | wrist_flex | **889** | 3133 | 197.2° | **890（贴下限，差 1）** |
| 5 | wrist_roll | 0 | 4095 | 359.9° | 2160（**出厂态，无限制**） |
| 6 | gripper | 1288 | 2713 | 125.2° | 2071（夹爪，行程本来就小） |

### 为什么能断定"被校准过"

ID 5 `wrist_roll` 是 **0~4095 的出厂默认值**，而其他关节都有收窄的区间
（跨度 193~212°，正好是机械结构的合理行程）。出厂不会这样，是有人设过的。

### 两个"异常"的真身

| 现象 | 真因 |
|---|---|
| ID 3 「一个 tick 都不动」 | 测试要动到 **3276**，但上限是 **3188**，超出 88 tick。舵机**直接忽略非法目标**，所以纹丝不动 |
| ID 4 「使能后漂移 +10 tick」<br>「回位失败，停在 890」 | 起始读数是 **874**，但下限是 **889**——**起点本身非法**。使能扭矩后舵机自己爬回合法区间（这就是那个"漂移"），回位想到 874 又被挡在 890 |

`扭矩上限 1000 / 扭矩限幅 1000`（ID 1~5 全满值，ID 6 夹爪 500 合理）——
**供电、扭矩、机械全都正常。**

### 判据注意

**不能只在"正好越过限位"时才报警。** ID 3 停在 3185、上限 3188，只差 3 tick，
严格判据会漏掉，但舵机已经吃不下任何正常测试幅度了。
`04_limits_check.py` 用 `LIMIT_MARGIN = 10` 处理这种情况。

### 已做的修复

- `02_joint_test.py`：现在先读软限位 → 把非法起点夹进合法区间 →
  正向被挡住时**自动改测反向** → 两边都顶死才判失败
- 新增 `04_limits_check.py`：只读的限位/扭矩诊断

### 姿态导致的"假故障"（ID 3 的完整排查过程，值得记）

限位问题修完后，ID 3 仍然失败。**换了三次假设才找对**，过程如下。

现象：ID 3 每次都**走 76~77 tick 就停**（目标 91 tick = 85%），
三次完全一致；而脚本改成失速检测后确认它**是真的停住不动**，不是超时。

```
3185 -> 3108   走了 77
3181 -> 3104   走了 77
3186 -> 3110   走了 76
```

**走过的弯路：**

| 假设 | 怎么排除的 |
|---|---|
| 速度太慢 / 时间不够 | ❌ 停点是**确定性**的，三次一致；改成失速检测后确认是"停下"而非"没走完" |
| 死区太大，舵机以为到位了 | ❌ 读了 `CW_Dead_Zone`/`CCW_Dead_Zone`，**全部 6 个关节都是 1 tick** |
| 扭矩限幅不够 | ❌ `Max_Torque_Limit` / `Torque_Limit` 都是 **1000**（满值） |
| 供电不足 | ❌ 12.5V，所有关节一致的正常窗口内 |

**决定性测量**：让脚本在失败时打印**停住瞬间的负载/电流**：

```
停住时的负载 72   电流 2   位置 3110
```

对比所有成功动作的负载：ID 5=`0`、ID 6=`-20`、ID 1=`24`、ID 4=`-28`、
ID 2=`-40`、**ID 3 卡住时=`72`（全场最大）**。

**负载 72 → 它在使劲顶，不是放弃不出力。** 电机/驱动/配置全是好的，
有东西在挡它。

**真因：姿态。** 当时肘部停在 `279.6°`，**紧贴校准范围上限**（3188），
前臂的位置让这个方向转不动。

**验证**：用手把手臂抬到"肘部在行程中段、前臂大致水平"，同样一条指令：

```
计划动作: 2340 -> 2431  (+8.0°)
[OK] 正向移动到位: 2431 tick  负载 -20  残差 +0 tick (+0.00°)
```

**负载从 72 掉到 -20，残差 0。** 换 6 个关节全测，6/6 通过。

**结论：二手机械臂上"某个关节转不动"，先怀疑姿态，再怀疑硬件。**
判别方法就是看停住瞬间的负载——大 = 有阻挡（多半是姿态/重力/干涉），
小 = 舵机没出力（配置或驱动问题）。

### 想真正放开行程（会覆盖前任的校准）

```bash
E:/lerobot-env/Scripts/python.exe 04_limits_check.py --port COM23   # 先读出来存好
```

改 EEPROM 限位需要先写 `Lock`(地址 55) = 0，改完写回 1。
**改之前务必先记录原始值**，否则丢了前任的校准就回不去了。

---

## 7. 主臂（COM32）为什么一开始用不了 lerobot

**根因：舵机 EEPROM 里的电压保护窗口被设窄了，不是硬件故障。**

2026-10-17 现象：`FeetechMotorsBus` 在 COM32 上握手直接失败

```
RuntimeError: FeetechMotorsBus motor check failed on port 'COM32':
Missing motor IDs: - 1 ... - 6
Full found motor list (id: model_number): {}
```

但同一个口用裸协议扫，6 个舵机全好（0 抖动、12.4~12.6V）。**矛盾点就是线索。**

### 真因

逐条 PING，发现每个应答包的**错误字节**都是 `0x01 输入电压错误`：

| | 允许电压窗口 | 实际电压 | 结果 |
|---|---|---|---|
| COM23 从臂 | 40~140 = **4.0~14.0V** | 12.5V | ✅ 窗口内 |
| COM32 主臂 ID 1,3,4,5,6 | 40~**120** = 4.0~**12.0V** | 12.4~12.6V | ❌ 超上限 |
| COM32 主臂 **ID 2** | 40~**80** = 4.0~**8.0V** | 12.6V | ❌ 超得离谱 |

12.5V 的供电压在 12.0V 上限之上 → 每个舵机都报电压错误 →
lerobot 的 `_assert_motors_exist` **把带错误标志的舵机整个过滤掉** →
"发现 0 个舵机" → `lerobot-calibrate` / `lerobot-teleoperate` / `lerobot-record` 全都起不来。

**关键：错误字节只是应答里的一个字节，不影响数据本身。** 裸协议完全能正常读写，
所以 `05_backup_calibration.py` 会在 lerobot 握手失败时**自动退回裸协议**。

### 修法

把窗口设回标准值（从臂就是这个值，也是 STS3215 出厂值）：

```bash
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py \
    --port COM32 --arm leader --id leader1 --voltage-window 40 140 --yes
```

改完 6 个舵机的 PING 错误字节全部变 0，lerobot 握手随即通过。

**这不影响安全性**：窗口只是"超出就报警"的阈值，真正的过压保护在电源侧；
12.5V 对 12V 标称的舵机完全正常（从臂的窗口本来就是 4.0~14.0V）。

### 教训

`Full found motor list: {}`（**一个都没有**）和"找不到某几个"是两回事：

| 现象 | 含义 |
|---|---|
| 少了某几个 ID | 那些舵机断线 / ID 被改过 / 接触不良 |
| **一个都没有，但裸协议能读** | 舵机在应答，但**带了错误标志**被 lerobot 过滤掉了 |

第二种情况别急着怀疑硬件，先看应答包里的错误字节。

---

## 8. 校准：不跑 lerobot-calibrate 也能用

### 为什么不能随便跑 lerobot-calibrate

`lerobot/motors/feetech/feetech.py` 的 `write_calibration()` 会**直接写舵机 EEPROM**：

```python
def write_calibration(self, calibration_dict, cache=True):
    for motor, calibration in calibration_dict.items():
        if self.protocol_version == 0:
            self.write("Homing_Offset", motor, calibration.homing_offset)
        self.write("Min_Position_Limit", motor, calibration.range_min)
        self.write("Max_Position_Limit", motor, calibration.range_max)
```

这三个地址就是第 6 节里那副"被前任校准过"的软限位。跑一次就全冲掉。

### 采用的方案：从 EEPROM 反向生成

`bus.is_calibrated` 的判据是 **「calibration 文件里的值 == 舵机 EEPROM 里的值」**：

```python
same_ranges  = 文件 range_min/max == EEPROM
same_offsets = 文件 homing_offset == EEPROM
return same_ranges and same_offsets
```

所以只要把 EEPROM 里已有的值写成 lerobot 要的 JSON，`is_calibrated` 直接为真——
**不碰舵机，前任的校准原封不动保留。**

```bash
# 1) 只读备份（会同时存原始记录 + lerobot 格式）
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py --port COM23 --arm follower --id follower1
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py --port COM32 --arm leader   --id leader1

# 2) 装成 lerobot 的 calibration 文件（不碰舵机）
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py --port COM23 --arm follower --id follower1 --install
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py --port COM32 --arm leader   --id leader1  --install
```

备份落在 `E:\lerobot-arm-test\calib_backup\`，三种文件：

| 文件 | 用途 |
|---|---|
| `<arm>_<id>_<时间戳>.json` | 完整记录（含备份时的位置/电压/温度/电压窗口），便于日后回看 |
| `<arm>_<id>_lerobot.json` | 纯 lerobot 格式，`--install` 就用它 |
| `<arm>_<id>.json` | 同上内容，固定文件名，`--restore` 默认引用它 |

万一被冲掉了可回滚：

```bash
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py \
    --port COM23 --arm follower --id follower1 --restore calib_backup/follower_follower1.json --yes
```

### 坑 1：校准路径比想当然的多一层

`robots/robot.py:47-53`：

```python
self.calibration_dir   = config.calibration_dir or HF_LEROBOT_CALIBRATION / ROBOTS / self.name
self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
```

**`self.name` 不是类型名，是类属性**：`SOFollower.name = "so_follower"`、
`SOLeader.name = "so_leader"`。所以真实路径是：

```
C:\Users\Administrator\.cache\huggingface\lerobot\calibration\robots\so_follower\follower1.json
C:\Users\Administrator\.cache\huggingface\lerobot\calibration\teleoperators\so_leader\leader1.json
```

早期按 `...\robots\follower1.json` 写过一次，lerobot 找不到文件 →
`is_calibrated` 返回 False → `connect()` 直接进了**交互式 calibrate()**
（在非交互环境里表现为 `EOFError: EOF when reading a line`）。

> 教训：**别按源码推断路径然后硬编码。** `05` 现在直接构造真类取
> `calibration_fpath`，让 lerobot 自己算。看到 `EOFError ... input()` 就是
> 走在 calibrate 分支上了，说明校准没被认出来。

### 坑 2：Homing_Offset 是 sign-magnitude，不是补码

`STS_SMS_SERIES_ENCODINGS_TABLE["Homing_Offset"] = 11` —— **符号位是 bit 11，幅值 0~10 位**。

实测对照（COM23，lerobot vs 裸协议）：

| 关节 | 原始字节 | 正确解码 | 按补码会解成 |
|---|---|---|---|
| shoulder_pan | 2406 | **−358** | 2406 |
| shoulder_lift | 2117 | **−69** | 2117 |
| gripper | 2071 | **−23** | 2071 |

`2406 & 0x7FF = 358` 且 bit11 置位 → −358。**正数碰巧对，负数全错**——
按补码解的话 `--restore` 会把垃圾值写回舵机。

修法是直接调用 lerobot 自己的 `encode_sign_magnitude` / `decode_sign_magnitude`
（`lerobot/motors/encoding_utils.py`），并做了逐位一致性验证：
COM23 上 `FeetechMotorsBus.read_calibration()` 与裸协议解码**完全一致**。

> 同一张表里 `Present_Load` 符号位 10、`Present_Position` 符号位 15，
> 所以第 6 节那个负载解码 `(raw & 0x3FF)` + `0x400` 符号位是对的。

---

## 9. 使能扭矩前必须对齐 Goal_Position（安全）

**舵机在 `Torque_Enable` 0→1 的瞬间会驱动到 `Goal_Position`。** 如果 Goal 和当前位置
差得远，机械臂就会蹿过去。

实测（2026-10-17，未处理前）：

| 臂 | 情况 |
|---|---|
| 从臂 COM23 | `shoulder_lift` 的 Goal 比当前位置差 **331 tick (29.1°)**；其余 5 个关节 ≤4 tick |
| 主臂 COM32 | **6 个关节的 Goal_Position 全是 0**，当前位置是 906~3074 —— 一旦使能就是 **270° 甩动** |

而 `SO101Follower.connect()` 跑完 `configure()` 后扭矩是**开着**的：

```python
# robots/so_follower/so_follower.py:160
with self.bus.torque_disabled():
    self.bus.configure_motors()
    ...

# motors/motors_bus.py:677-691 —— 注意 finally 里会重新使能
def torque_disabled(self, motors=None):
    self.disable_torque(motors)
    try:
        yield
    finally:
        self.enable_torque(motors)
```

所以从臂一 connect 完就使能扭矩，那 29° 会真的蹿出来。

**做法**：使能扭矩前先把 `Goal_Position`(地址 42) 写成自己的 `Present_Position`(地址 56)。
因为写完之后 Goal == Present，不管扭矩什么时候开，臂都停在原地。

### 9.1 坑：写 `Goal_Position` 会自动把 `Torque_Enable` 置 1

这一点官方文档没写，各驱动库也都把两者当成独立寄存器处理，所以很容易漏掉。

**实测（2026-10-17，主臂 COM32，ID 1）**：

```
BEFORE torque = 0
present = 1121   goal = 1380
写 Goal_Position := 1121
AFTER  torque = 1      ← 只写了 Goal，扭矩自己开了
restored torque = 0
```

**后果**：

- **`06_prime_goal.py --yes` 执行完，扭矩就已经开着、臂是硬的**，不是"扭矩还是关的"。
  这一点之前的文档写错了，已更正。
- 但仍然安全：写入时 Goal := 当场读到的 Present，所以使能瞬间不会动。
- 所以"先对齐再连 lerobot"依然成立，只是原因不是"扭矩没开"，而是 **Goal 等于当前位置**。
- 反过来说：**只要不打算马上用，就不要随便写 Goal**，否则臂会自己锁死、持续发热。

验证方法（任何时候都能复现）：

```bash
E:/lerobot-env/Scripts/python.exe -c "
import time
from lerobot.motors.feetech.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
m = {'shoulder_pan': Motor(1,'sts3215',MotorNormMode.DEGREES)}
bus = FeetechMotorsBus(port='COM32', motors=m); bus.connect(handshake=False)
print('before', bus.read('Torque_Enable','shoulder_pan'))
p = bus.read('Present_Position','shoulder_pan',normalize=False)
bus.write('Goal_Position','shoulder_pan',p,normalize=False); time.sleep(0.3)
print('after ', bus.read('Torque_Enable','shoulder_pan'))
bus.write('Torque_Enable','shoulder_pan',0)
bus.disconnect(disable_torque=False)
"
```

### 9.2 断电重上电之后

`Torque_Enable` 和 `Goal_Position` **都在 SRAM 里**，断电全部复位：

| 寄存器 | 断电后 | 含义 |
|---|---|---|
| `Torque_Enable` | 0 | 臂是松的，可以自由扳动 |
| `Goal_Position` | **0** | 不是当前位置！ |

实测（从臂 COM23 断电重上电后）：6 个关节 Goal 全为 0，Present 在 1967~3919。
此时若直接连 lerobot，`configure()` 使能扭矩 → 6 个关节同时冲向 0 位，**最大 344°**。

所以断电重启后的顺序必须是：

```
断电 → 上电（扭矩自然为 0）→ 06_prime_goal.py --yes → 再连 lerobot
```

```bash
E:/lerobot-env/Scripts/python.exe 06_prime_goal.py --port COM23          # 只看差值
E:/lerobot-env/Scripts/python.exe 06_prime_goal.py --port COM23 --yes    # 写入
```

这就是 `02_joint_test.py` 一直遵守的顺序（读位置 → 目标:=当前位置 → 再使能），
也是**每次跑 lerobot 前都该做的一步**。

> 主臂本身安全（`SOLeader.configure()` 只 `disable_torque()`，主臂设计上不使能扭矩），
> 但它的 Goal 全是 0 是个雷——任何误使能就是 270° 甩动。顺手对齐掉。

### lerobot 连接时会写什么、不写什么

`SO101Follower.configure()` 会写：`Return_Delay_Time` / `Acceleration` /
`Maximum_Acceleration` / `Phase` / `Operating_Mode` / P-I-D 增益
（+ 从臂夹爪的 `Max_Torque_Limit=500` / `Protection_Current=250` / `Overload_Torque=25`）。

**`Homing_Offset` / `Min_Position_Limit` / `Max_Position_Limit` 一个都不碰**——
校准不会被 connect 破坏，这是它每次运行都做的常规配置。

> 顺带一个佐证：从臂夹爪的 `Max_Torque_Limit` 实测就是 500，
> 正是 lerobot `configure()` 写进去的值 —— 说明前任用的也是 lerobot。

---

## 10. 整机验收

```bash
E:/lerobot-env/Scripts/python.exe 07_verify_lerobot.py --follower-port COM23 --leader-port COM32
```

用 lerobot 真实的 `SO101Follower` / `SO101Leader` 类连接，检查 `connect()`、
`is_calibrated`、`get_observation()` / `get_action()`。

实测结果（2026-10-17）：

```
从臂 COM23:  connect() -> OK   is_calibrated -> True   get_observation() -> 6 个关节   [OK]
主臂 COM32:  connect() -> OK   is_calibrated -> True   get_action()      -> 6 个关节   [OK]
```

下一步遥操作：

```bash
E:/lerobot-env/Scripts/lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=COM23 --robot.id=follower1 \
  --teleop.type=so101_leader --teleop.port=COM32 --teleop.id=leader1
```

---

## 11. 其他

- **pip 缓存已占 18GB**（C: 盘）。这些是历史下载缓存，可以安全清理：
  `E:/lerobot-env/Scripts/python.exe -m pip cache purge`
  （注意这会清掉所有 venv 共享的缓存，下次装包要重新下）
- C: 剩 114G / D: 剩 55G / E: 剩 1.2T —— 模型和数据都放 E:
