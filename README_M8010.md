# 宇树 GO-M8010-6 电机 CAN 控制示例

本目录下与 GO-M8010-6 相关的三个文件，用本仓库的 USB2CANFD 设备 + 宇树「CAN转485 模块」
驱动 GO-M8010-6 关节电机。

| 文件 | 作用 |
| --- | --- |
| [`go_m8010_can.py`](demo/go_m8010_can.py) | 协议封装：组 29 位扩展帧、定点编解码、参数设置与回读、上传帧解析 |
| [`test_motor.py`](demo/test_motor.py) | 接线自检：设/读 Kpos·Kspd 并打印上传帧，**默认不会让电机转动** |
| [`sine_follow.py`](demo/sine_follow.py) | 应用示例：速度（默认）或位置正弦跟随，带渐入、误差统计与报错中止 |

协议出处：<https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/CAN_to_485>
（页面是 SPA，正文在 `https://doc-cdn.unitree.com/8/472/zh/8_472_zh`）

---

## 1. 硬件连接

```
PC ──USB──> USB2CANFD ──CAN_H/CAN_L──> CAN转485 模块 ──485──> GO-M8010-6
```

CAN 侧的设置：

| 项 | 说明 |
| --- | --- |
| 模块 ID | 拨码开关前两位，0~3，出厂默认 **3** |
| CAN 波特率 | **1M**（拨码第 4 位 OFF，当前模块已确认） |
| 帧类型 | **29 位扩展帧**，经典 CAN，DLC = 8 |
| 终端电阻 | 总线两端各 120Ω。USB2CANFD 支持软件开关，脚本会自动打开 |
| 模块供电 | 24V，注意加速时的瞬时电流 |

软件终端电阻是**会话级**设置，设备重新枚举后要重新下发——`test_motor.py` 和
`sine_follow.py` 每次运行都会自动打开。

---

## 2. 安装

```bash
# 装 SDK 的 Python 包（本仓库 Python/pack 下自带 wheel）
pip install pack/dmcan_sdk-2.1.1-py3-none-any.whl
```

后端动态库由脚本自动查找（环境变量 `DMCAN_LIBRARY_PATH` 优先）：

- Windows：`dlls/libdmcan.dll`
- Linux：`C&C++/lib/v2.1.0/linux/<x86_64|arm64>/libdmcan.so`

---

## 3. 快速开始

以下命令都在 `demo/` 目录下执行：

```bash
cd demo
```

### 3.1 先自检（电机不动）

```bash
python3 test_motor.py
```

正常输出：

```
已打开通道 0 的软件终端电阻
下发：模块 3 / 电机 0，设 Kpos=κp*1280=128, Kspd=κw*1280=256
RX ch=0 id=0x1E020080 ext=1 dlen=8 0001800000000000
   -> 参数回读：module=3 motor=0 ... Kspd=256(κw=0.200) Kpos=128(κp=0.100)
共收到 1 帧，通信正常。
```

收到回读帧就说明：模块 ID、电机 ID、波特率、接线全部正确。

常用参数：`--module-id 3 --motor-id 0 --channel 0`（波特率沿用设备里保存的配置；当前模块为
**1M**，若通道配置不是 1M，可加 `--bitrate 1000000` 显式设置）。
`--move` 会额外做一次零位置低刚度保持（**电机会动**）。

### 3.2 速度正弦跟随

```bash
python3 sine_follow.py                              # 出轴 ±6.28 rad/s、0.2 Hz、10 秒
python3 sine_follow.py --amplitude 1.0 --freq 0.1   # 小幅度
python3 sine_follow.py --kd 0.02                     # 跟得更紧
```

关键参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--mode` | `velocity` | `velocity` 速度跟随（kp 强制 0）；`position` 位置跟随 |
| `--amplitude` | 速度 6.28 rad/s / 位置 0.5 rad | 出轴正弦峰值 |
| `--freq` | 0.2 | 正弦频率 Hz |
| `--duration` | 10 | 运行时长 s |
| `--ramp` | 1.0 | 幅度渐入时间 s |
| `--kp` / `--kd` | 0.03 / 0.01·0.05 | 与宇树 SDK 的 `cmd.kp` / `cmd.kd` 同口径 |
| `--gear-ratio` | 6.33 | GO-M8010-6 减速比 |
| `--rate` | 200 | 控制帧频率 Hz |
| `--raw-gains` | 关 | 不做口径换算，直接把 kp/kd 当转子侧 κ 下发 |
| `--no-termination` | 关 | 不自动打开软件终端电阻 |

结束或 Ctrl-C 时会把速度归零再锁定。

---

## 4. 协议摘要

### 4.1 CAN ID（29 位扩展帧）

| 位 | 字段 |
| --- | --- |
| 28-27 | 模块 ID（0~3） |
| 26 | 方向：0 = 下发，1 = 上传 |
| 25-24 | 内容：0 = 电机控制，1 = 电机响应，2 = 参数回读 |
| 23-16 | 控制模式（`低位3`）：10 / 11 / 12 / 13 |
| 15-8 | `低位2` = 电机 ID(bit 0-3) + 工作状态(bit 4-6) |
| 7-0 | `低位1`：下发预留；上传时为温度（=-128 表示报错） |

控制模式：

| 模式 | 含义 |
| --- | --- |
| 10 | 下发控制量，且**每帧都回**一帧数据（mode1） |
| 11 | 设置 Kpos / Kspd（模块断电丢失，重新上电要重发） |
| 12 | 回读 Kpos / Kspd（应答内容 = 2） |
| 13 | 下发控制量，**默认不回**，仅报错时回（mode2） |

工作状态：`0` 锁定 / `1` FOC 闭环 / `2` 编码器校准（等待 5s，期间禁止发任何数据包）。

### 4.2 数据区（8 字节，小端）

| 模式 | 字节布局 |
| --- | --- |
| 10 / 13 | 位置 int32 @0-3、速度 int16 @4-5、力矩 int16 @6-7 |
| 11 | 阻尼 Kspd uint16 @0-1、刚度 Kpos uint16 @2-3（**阻尼在前**） |
| 12 | 全 0 |

上传（内容 = 1）：实际位置 int32 @0-3、实际速度 int16 @4-5、实际力矩 int16 @6-7，
`低位1` = 温度，`低位3` = 气压（8 bit 压缩值）。

### 4.3 定点换算

| 物理量 | 编码 | 范围 |
| --- | --- | --- |
| 位置 | `counts = rad/(2π) × 32768`（int32） | ±411774 rad |
| 速度 | `counts = (rad/s)/(2π) × 256`（int16） | ±804.0 rad/s |
| 力矩 | `counts = N·m × 256`（int16） | ±127.99 N·m |
| Kpos / Kspd | `counts = κ × 1280`（uint16） | 0 ~ 25.599 |

以上都是**电机转子侧**（减速器之前）的数值。出轴量 = 转子量 ÷ 6.33。

---

## 5. 单位与增益口径（重要）

宇树 SDK 的 `cmd.q` / `cmd.kp` / `cmd.kd` 是**出轴侧**口径，而 CAN 模块要的是转子侧：

| 量 | 转子侧下发值 |
| --- | --- |
| 位置 | 出轴 rad × 6.33 |
| 速度 | 出轴 rad/s × 6.33 |
| 位置模式 kp / kd | 出轴值 **× 6.33²**（≈40） |
| 速度模式 kd | **不放大**，直接下发（官方速度例程就是 `kd=0.01`） |

`go_m8010_can.py` 只做定点换算，不管减速比；口径换算在 `sine_follow.py` 里
（`--raw-gains` 可关闭）。

官方参考参数：

```python
# 位置模式
cmd.q  = 6.28 * gear_ratio
cmd.dq = 0.0
cmd.kp = 0.01
cmd.kd = 0.01
cmd.tau = 0.0

# 速度模式（kp 与 tau 必须为 0）
cmd.q  = 0.0
cmd.dq = 6.28 * gear_ratio
cmd.kp = 0.0
cmd.kd = 0.01
cmd.tau = 0.0
```

---

## 6. API 速览（`go_m8010_can.py`）

```python
from go_m8010_can import GoM8010Can

motor = GoM8010Can(device, channel=0, module_id=3, motor_id=0)

motor.set_gains(kp=0.4, kd=0.01)   # 模式 11：转子侧 κ，模块断电丢失
motor.send(position=0.0,            # 模式 10：位置/速度/力矩（转子侧）
           velocity=39.75,          # quiet=True 改用模式 13，不回帧
           torque=0.0,
           status=GoM8010Can.STATUS_FOC)
motor.read_gains()                  # 模式 12
motor.lock()                        # 状态 0 + 控制量全零
```

上传帧解析（回调里用）：

```python
info = motor.parse_upload(frame.can_id, bytes(frame.payload[:frame.dlen]))
# info 为 None：不是本模块/本电机的上传帧
# {"kind": "motor", "position", "velocity", "torque", "temperature", "pressure"}
# {"kind": "error", "error", "temperature"}
# {"kind": "gains", "kp", "kd"}
```

另有正反换算的静态方法：`position_to_counts` / `velocity_to_counts` / `torque_to_counts`
与 `counts_to_*`。

---

## 7. 实测记录

硬件：USB2CANFD_S + CAN转485（模块 ID 3）+ GO-M8010-6（电机 ID 0），1 Mbps。

| 项目 | 结果 |
| --- | --- |
| 模式 12 回读 | 应答 `id=0x1E020000`，数据 `0001 8000` → 证实小端、Kspd 在前 |
| 位置跟随（κp=0.401, κw=0.401） | 跟随滞后约 0.3~0.5 s，幅度到目标的 70~80% |
| 速度跟随（κw=0.01，出轴 ±6.28 rad/s @0.2 Hz） | 滞后约 60 ms，误差 RMS ≈ 8% |
| 力矩量级 | 速度跟随稳态约 0.03~0.04 N·m |

---

## 8. 常见问题

**收不到任何上传帧**

1. 模块拨码 ID 是否等于 `--module-id`（出厂 3）；
2. 波特率：拨码第 4 位 OFF = 1M（本机已确认 1M），是否与通道配置一致；
3. 电机 ID 是否等于 `--motor-id`；
4. 总线终端电阻、CAN_H / CAN_L 是否接反；
5. 通道是否已使能（脚本会做）。

**电机报错**

上传帧 `低位1 = -128` 时表示报错，此时 `低位3` 是错误代号：

| 代号 | 含义 |
| --- | --- |
| 0 | 正常 |
| 1 | 过热 |
| 2 | 过流 |
| 3 | 过压 |
| 4 | 编码器故障 |
| 5 | 母线欠压 |
| 6 | 绕组过热 |
| 7 | 未标定 |

错误 5（母线欠压）会在加速电流较大时触发，之后**锁存**——需要断电重上电，
并检查 24V 电源的电压与电流能力。

**电机不动 / 只是轻微响**

- 速度模式 `kd` 很小，幅度太低（出轴 <1 rad/s）时力矩不足以克服摩擦；
- 位置模式下 κp 太小同理，适当加大 `--kp`；
- 确认用的是模式 10/13 以外没有漏发模式 11。

**上电后参数失效**

模式 11 设置的 Kpos/Kspd 存在模块里，**模块断电即丢**，重新上电后要先 `set_gains()`。
