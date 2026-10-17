# 项目说明（自动加载）

这个文件在每个新会话里自动读取。**内容是稳定的项目事实，不是进度记录**——
进度看 `PROGRESS.md`，逐周任务看 `plan.html`。

---

## 这是什么

用户的 SO-101 主从机械臂测试与学习项目，以及一条 24 周的学习路线：
从「纯视觉检测 + C# 上位机」延伸到「工业视觉 + 运动控制 + 边缘部署」。

用户背景：Cognex VisionPro + C# 上位机、总线型/脉冲型运动控制板卡、精密测量。
**不是 AI 背景，Python 和深度学习从零开始。回答要偏实操，给具体命令和失败模式。**

## 硬件与端口（实测）

| 设备 | 说明 |
|---|---|
| SO-101 从臂 | **COM23**，id=`follower1`，12.4~12.5V |
| SO-101 主臂 | **COM32**，id=`leader1` |
| 舵机 | Feetech STS3215，1 Mbps，ID 1~6 |
| D435i | SN `241222073855`，固件 `5.17.3.10`，USB 3.2，含 IMU。`pyrealsense2` 装在 `E:\lerobot-env`（2.59.1），**没装独立 SDK** |
| Jetson | **型号待确认**（Orin Nano 8GB / 原版 Nano） |
| RTX 4070 SUPER | 12 GB，训练与重建 |
| Python 环境 | **`E:\lerobot-env\Scripts\python.exe`**（venv，基于 Anaconda 3.12.7），lerobot 0.6.1、torch 2.11+cu128、pyrealsense2 2.59.1 都在这。**系统 Python `...\Programs\Python\Python312` 里什么都没有，不要用** |

工作目录 `E:\lerobot-arm-test`。lerobot 源码在
`E:\lerobot-env\Lib\site-packages\lerobot\`（不是 git clone）。

## 已经踩过的坑（不要再犯）

1. **主臂曾完全用不了 lerobot**：EEPROM 电压保护窗口设成 4.0~12.0V（ID 2 是 4.0~8.0V），
   供电 12.5V 压在其上，应答包带「输入电压错误」，lerobot 握手把带错误标志的舵机全过滤掉。
   已改成 4.0~14.0V。**判别法：一个都没找到但裸协议能读 → 查应答包错误字节，别怀疑硬件。**

2. **`lerobot-calibrate` 直接写 EEPROM** 的 `Homing_Offset`/`Min_Position_Limit`/`Max_Position_Limit`，
   会冲掉前任校准。改之前必须 `05_backup_calibration.py` 备份。

3. **使能扭矩前必须把 `Goal_Position` 写成 `Present_Position`**，否则舵机会冲向历史目标。
   `SO101Follower.connect()` 的 `configure()` 跑完扭矩是开着的。

4. **写 `Goal_Position` 会自动把 `Torque_Enable` 置 1**（固件行为，文档没写）。
   所以 `06_prime_goal.py --yes` 跑完臂是硬的。不打算马上用就别随便写 Goal。

5. **断电会清零 `Goal_Position`（SRAM），复位成 0，不是当前位置。**
   顺序必须是：断电 → 上电 → `06_prime_goal.py --yes` → 再连 lerobot。

6. `Homing_Offset` 是 **sign-magnitude（符号位 bit 11）不是补码**。
   按补码解正数碰巧对、负数全错。

7. 校准文件路径是 `.../calibration/robots/**so_follower**/<id>.json`，
   有 `so_follower` 这一层（那是类的 `name` 属性，不是类型名）。

8. **D435i：`np.asanyarray(frame.get_data())` 是零拷贝视图，持有它就持有整个 frame。**
   攒满帧池（D435i 是 **16**）之后 `align.process()` 会报
   `Error occured during execution of the processing block`，
   而且是在第 16 帧才崩，很容易误判成"场景问题"或"驱动问题"。
   **存数组一律用 `np.array(...)` 或 `.copy()`。**
   （`pipe.start()` 的返回值也要接住，否则 profile 被回收。）

## 工作原则

- **读操作可以自由执行；任何写舵机 EEPROM 或改变机器人状态的操作，先说明再动手。**
- 涉及机械臂的调试先查 `ENVIRONMENT.md`。
- 修改校准/限位/电压窗口前先备份。
- 每轮 lerobot 运行前先 `06_prime_goal.py`；进程被强杀后用 `--release` 收尾。
- Windows + Git Bash 环境，命令写成单行，避免粘贴断行。

## 文件地图

| 文件 | 用途 |
|---|---|
| `plan.html` | **执行入口**：24 周逐项任务、勾选进度、甘特图 |
| `PROGRESS.md` | **进度与决策记录**，新会话先读这个 |
| `ROADMAP.md` | 总纲：为什么这么排 |
| `LEARNING_PLAN.md` | lerobot 模组展开 |
| `3DGS_DEFECT_SYNTHESIS.md` | 3DGS 缺陷合成展开 |
| `ENVIRONMENT.md` | 本机实测记录（最权威的事实来源） |
| `README.md` | 脚本用法与分层验证流程 |
| `01`~`07_*.py` | 裸协议诊断/测试/备份/对齐/验收脚本 |
| `calib_backup/` | 校准备份 |
| `vision/` | D435i 采集与深度诊断脚本（第 1 周起） |
| `vision/captures/` | 采集输出：color / depth / overlay / intrinsics.json |

## 回答风格

- 中文，偏工程落地，给具体数字和命令。
- 明确说"不能做什么"，尤其是精度边界（D435i 不是计量级）。
- 用户做精密测量出身，对"方法的优化目标决定精度上限"这类判断有共鸣。
