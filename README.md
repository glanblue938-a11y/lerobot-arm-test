# SO-101 机械臂功能测试

二手设备、状态未知，所以按**从下往上分层验证**：每一层过了再往上，出问题才能定位。

核心原则：**先只读，后运动；先单关节，后整机。**

---

## 目录

```
E:\lerobot-arm-test\
├── README.md              本文件
├── ENVIRONMENT.md         本机实测环境记录（限位/供电/FFmpeg/电压窗口/校准的实测数据）
├── 01_scan_bus.py         只读诊断：不发任何写指令，机械臂不会动
├── 02_joint_test.py       单关节受控运动测试（会让机械臂动）
├── 03_power_check.py      供电诊断，只读
├── 04_limits_check.py     软限位/扭矩诊断，只读
├── 05_backup_calibration.py  校准备份 / 安装 / 恢复，兼电压窗口修复
├── 06_prime_goal.py       使能扭矩前对齐 Goal_Position（安全，必做）
├── 07_verify_lerobot.py   用 lerobot 真实的类做整机验收
├── setup_ffmpeg_shim.py   让 torchcodec 找到 FFmpeg（一次性，装完就不用管）
├── calib_backup/          05 生成的备份
└── _selftest.py           离线自检，验证脚本本身没写错。你不用跑
```

### 05 / 06 / 07 什么时候用

| 脚本 | 什么时候 | 会写舵机吗 |
|---|---|---|
| `05 ...`（不带参数） | 每次要动校准**之前**。只读备份 | 否 |
| `05 ... --install` | 想把 EEPROM 里已有的校准变成 lerobot 的文件 | 否（只写文件） |
| `05 ... --voltage-window 40 140 --yes` | lerobot 报「发现 0 个舵机」时 | 是（电压窗口） |
| `05 ... --restore <备份> --yes` | 校准被误覆盖，要回滚 | 是（校准三个寄存器） |
| `06_prime_goal.py ... --yes` | **每次跑 lerobot 连接机械臂之前** | 是（只写 SRAM 的 Goal_Position） |
| `06_prime_goal.py ... --release --yes` | lerobot 进程被**强杀**后收尾（扭矩会留在开启状态） | 是（关扭矩） |
| `07_verify_lerobot.py ...` | 想知道 lerobot 到底能不能用这台臂 | 是（lerobot 的常规运动参数，不碰校准） |

依赖只有一个：

```bash
pip install pyserial
```

---

## L0 · 上电前的物理检查（不用代码，但别省）

- [ ] 12V 电源接好，主臂（leader）和从臂（follower）**各自独立供电**
- [ ] 末端相机的 USB 走独立接口，别和舵机总线抢
- [ ] **手放在电源开关上**
- [ ] 上电瞬间，6 个关节应同时"锁死"变硬，有轻微咔哒声
      → 哪个关节是软的、还能用手扳动，那个电机就没上电或 ID 不对
- [ ] 用手扳锁死的关节，检查有没有明显卡涩、异响、旷量（齿轮箱问题在这一步就能发现）

> 二手设备特别提醒：先目视确认接线端子有没有松动、舵机线有没有被夹伤过。
> 总线舵机是菊花链，**中间任何一个接头接触不良，后面所有舵机一起掉线**，
> 表现会非常像"电机坏了"，其实只是接头松了。

---

## L1 · 找串口

```bash
python 01_scan_bus.py --list
```

这一步会列出所有串口。**关键技巧**：拔掉机械臂的 USB 再看一遍，消失的那个才是舵机总线。

> 你这台机器上有 20 多个蓝牙虚拟串口（COM16~COM31），非常容易选错。
> 真实舵机总线是 CH340 芯片，描述里会写 `USB-SERIAL CH340`。

---

## L2 · 只读诊断（最重要的一步）

```bash
python 01_scan_bus.py --port COM7
```

**不确定波特率时**（二手设备很可能被改过）：

```bash
python 01_scan_bus.py --port COM7 --all-baud
```

### 这个脚本会告诉你什么

| 输出项 | 说明 |
|---|---|
| 波特率 | 逐个试，命中会自动报告 |
| 舵机 ID 列表 | 暴力扫 0~20，**不是**读 ID 寄存器（那个地址在不同固件版本会偏移，不可靠） |
| ID 是否齐全 | 应该正好是 1~6，多一个少一个都要查 |
| 每个关节读数 | 位置 / 电压 / 温度 / 负载 / 扭矩使能状态 |
| 通信稳定性 | 每个 ID 连读 6 次位置，看抖动。抖动 > 30 tick 就是 ID 冲突或供电噪声 |

### 正常应该看到

```
  发现舵机: 6 个  ->  ID [1, 2, 3, 4, 5, 6]
  [OK] ID 集合完全符合 SO-101 出厂配置（1~6）
  [OK] 未发现异常
```

### 常见异常对照

| 现象 | 原因 | 怎么办 |
|---|---|---|
| 一个舵机都没有 | 波特率不对 / 总线板没供电 / COM 口选错 | `--all-baud`；USB 供电通常不够，必须接主电源 |
| 少了某个 ID | 该舵机 ID 被改过，或排线断 | 单独把那个舵机接到总线板上，用厂商上位机确认 ID |
| 多出 ID 7、8… | 前一位使用者重编过号 | 全部重设成 1~6（用 Feetech 官方上位机，注意一次只接一个） |
| 位置读数跳变很大 | **ID 重复**（两个舵机同号，应答互相打架） | 一次只接一个舵机，逐个确认 ID |
| 电压低于舵机自己的下限 | 供电不足 | 换电源；注意舵机堵转瞬间电流很大。用 `03_power_check.py` 读 EEPROM 地址 14/15 拿到该舵机**自己的**允许窗口，别套用通用标称值 |
| 电压读数恒定不变 | 通常只是电源稳，**不一定是地址错** | 看同一批采样里温度是否变化：会变就说明读取链路正常。地址 62 已由 `lerobot/motors/feetech/tables.py` 确认 |
| 温度读数 > 100 或为负 | 该寄存器地址和你的固件不匹配 | 以 lerobot 源码里的表为准，重新核对地址 |

---

## L3 · 单关节运动测试

**先 dry-run 看计划，不动手**：

```bash
python 02_joint_test.py --port COM7 --id 1 --dry-run
```

确认无误后，一次测一个：

```bash
python 02_joint_test.py --port COM7 --id 1 --yes
```

6 个关节依次测（中间有 3 秒间隔）：

```bash
python 02_joint_test.py --port COM7 --id all --yes
```

### 这个脚本的安全设计

使能扭矩时**不会让关节"弹"到别处**。顺序是：

```
读当前位置  ->  把目标位置写成当前位置  ->  再使能扭矩
```

这样电机上电瞬间锁在原地，而不是猛地冲向某个历史目标值。
**这个顺序是脚本存在的核心意义，别自己改成别的顺序。**

其他保护：

- 必须显式传 `--yes` 才会执行，否则直接拒绝
- `--delta` 限制在 0~30 度，默认 ±8 度
- 目标位置超出 0~4095 就跳过（接近行程极限）
- 不管成功失败，退出前一定关闭扭矩（`finally` 保证）
- 正反各走一趟，验证双向都能动

### 每个关节应该看到

```
  [OK] 使能扭矩后位置 2048 tick（相对起始漂移 +2 tick）
  [OK] 正向移动到位: 2139 tick  负载 87
  [OK] 反向回到原位: 2048 tick
  温度: 32°C -> 32°C
  已关闭扭矩，关节现在可以用手自由扳动
```

### 测出问题怎么判断

| 现象 | 可能原因 |
|---|---|
| 上电瞬间漂移 > 15 tick | 目标寄存器没写进去 / 该关节负载很重 / 减速箱打滑 |
| 移动不到位，停在半路 | 扭矩不足 / 机械卡涩 / 被限位挡住 / 负载太重 |
| 负载读数异常大（接近 1000） | 机械卡死或装配过紧 |
| 动了一下温度明显上升 | 内部摩擦大，拆开检查 |
| 能往一个方向动，反向不动 | 限位寄存器被写过 / 该方向机械卡死 |
| 完全不动但读数正常 | `Torque_Enable` 没生效，或电机驱动级故障 |

---

## L4 · 装 LeRobot 环境

前面三步都不依赖 lerobot，用的是裸串口协议。到这里才需要装。

> ⚠️ **不要直接 `pip install lerobot`**。这个项目版本迭代很快，命令名和数据集格式
> 在 0.4 之后改过好几轮，网上教程大概率和你装到的版本对不上。
> 建议 clone 源码锁定版本。

```bash
conda create -n lerobot python=3.12 -y
conda activate lerobot

git clone https://github.com/huggingface/lerobot.git E:/lerobot
cd /e/lerobot

# 先看一眼 extras 到底叫什么名字，别照抄网上的
grep -A 30 "optional-dependencies" pyproject.toml

pip install -e ".[feetech]"
```

> 上面 `feetech` 这个 extra 名字请以你 clone 到的 `pyproject.toml` 为准。
> 有些版本叫 `[all]`，有些把训练依赖拆成了 `[training]`。

---

## L5 · 校准 + 遥操作（整机功能验收）

这一步才是最直观的"整机功能测试"：**摇杆带动从臂，所有关节跟不跟、夹爪开合顺不顺、有没有卡顿和异响。**

### 先跑这三步（顺序别换）

```bash
# 1) 只读备份校准 —— 每次动手之前都先做
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py --port COM23 --arm follower --id follower1
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py --port COM32 --arm leader   --id leader1

# 2) 使能扭矩前对齐 Goal_Position（安全，必做）
E:/lerobot-env/Scripts/python.exe 06_prime_goal.py --port COM23 --yes
E:/lerobot-env/Scripts/python.exe 06_prime_goal.py --port COM32 --yes

# 3) 验收 lerobot 到底能不能用
E:/lerobot-env/Scripts/python.exe 07_verify_lerobot.py --follower-port COM23 --leader-port COM32
```

> 第 2 步不是可选的。舵机在 `Torque_Enable` 0→1 的瞬间会驱动到 `Goal_Position`，
> 而 `SO101Follower.connect()` 跑完 `configure()` 后扭矩是**开着**的。
> 实测对这副二手臂：从臂 `shoulder_lift` 差 29°，主臂 6 个关节的 Goal **全是 0**（差 270°）。

### 校准：别急着 `lerobot-calibrate`

`lerobot-calibrate` 会**直接写舵机 EEPROM** 的 `Homing_Offset` / `Min_Position_Limit` /
`Max_Position_Limit` —— 也就是把前任的校准全冲掉。

如果臂本身已经被校准过（`04_limits_check.py` 能看出来），更好的路是
**把 EEPROM 里已有的值反向生成 lerobot 的 calibration 文件**：

```bash
E:/lerobot-env/Scripts/python.exe 05_backup_calibration.py \
    --port COM23 --arm follower --id follower1 --install
```

`is_calibrated` 的判据就是「文件里的值 == EEPROM 里的值」，所以装上就直接通过，
**不用跑 calibrate，也不用碰舵机**。实测两台臂都 `is_calibrated -> True`。

真要按标准流程重新校准（比如零位定义不满意）也可以，但**先跑第 1 步备份**：

```bash
E:/lerobot-env/Scripts/lerobot-calibrate --robot.type=so101_follower --robot.port=COM23 --robot.id=follower1
E:/lerobot-env/Scripts/lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM32 --teleop.id=leader1
```

### 遥操作

```bash
E:/lerobot-env/Scripts/lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=COM23 --robot.id=follower1 \
  --teleop.type=so101_leader --teleop.port=COM32 --teleop.id=leader1
```

**验收标准**：手动摆动主臂，从臂能实时跟随，没有明显延迟、抖动、异响；
夹爪开合跟得上；把主臂摆到各个极限位置，从臂不撞限位。

### 校准这一步的坑

- 校准时每个关节要**手动推到中位再按回车**，主从臂的中位定义必须一致
- 校准时如果某个关节扳不动，说明扭矩还使能着，先断电重上电
- **看到 `EOFError: EOF when reading a line` 就是走在交互式 calibrate 分支上了**
  —— 说明校准文件没被认出来，先查路径（见 ENVIRONMENT.md 第 8 节「坑 1」），
  别以为是在报别的错
- 校准文件路径是 `.../calibration/robots/so_follower/<id>.json`，
  **有 `so_follower` 这一层**，不是 `.../robots/<id>.json`

---

## L6 · 录制 + 回放（数据链路验收）

```bash
lerobot-record \
  --robot.type=so101_follower --robot.port=COM7 --robot.id=follower1 \
  --teleop.type=so101_leader --teleop.port=COM8 --teleop.id=leader1 \
  --dataset.repo_id=<你的HF用户名>/test_pick_place \
  --dataset.num_episodes=3 \
  --dataset.single_task="把方块放进盒子"
```

回放验证：

```bash
lerobot-replay --robot.type=so101_follower --robot.port=COM7 --robot.id=follower1 \
  --dataset.repo_id=<你的HF用户名>/test_pick_place --dataset.episode=0
```

**验收标准**：回放时机械臂动作和你录制时基本一致；
数据集目录里视频和关节数据都落盘了，帧数对得上。

眼在手上配置额外检查：录制时机械臂运动状态下，末端相机画面**不能有撕裂或掉帧**。
有的话降分辨率或换 USB 口。

---

## 出问题时的排查顺序

卡住了就按这个顺序回溯，别跳：

1. **L0** 供电和接线（最容易被忽略，也最容易出问题）
2. **L1** COM 口对不对（你机器上一堆蓝牙虚拟串口，踩坑率极高）
3. **L2** 波特率和 ID
4. **L3** 单关节能不能动
5. **L4** 环境装没装对（`pip show lerobot`、`lerobot-calibrate --help` 看命令存不存在）
6. **L5** 校准文件是不是这一台的

---

## 关于脚本本身的可靠性

`01` 和 `02` 里的寄存器地址来自 Feetech STS 系列的通用表，协议组包逻辑已经用
标准包 `FF FF 01 02 01 FB` 离线验证过（跑 `python _selftest.py`）。

但**寄存器地图在不同固件版本上可能有偏移**，脚本无法替你验证这一点。
所以它给每个读数都加了合理性区间检查（温度 0~100、电压范围、位置 0~4095），
地址对不上时会报"读数可疑"而不是当成正常值放过去。

唯一有写入风险的参数是 `--speed`（写速度寄存器）。**默认不写**，
不确定就别传。运动幅度靠 `--delta` 控制已经足够安全。
