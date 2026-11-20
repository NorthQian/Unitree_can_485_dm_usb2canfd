# 泉智博（QZB）电机 CAN FD 控制示例

本目录下与泉智博电机相关的文件，用本仓库的 USB2CANFD 设备以 **CAN FD + MIT 协议**
驱动泉智博伺服关节。协议参考 `Noetix_motorevo_mc02` 工程（STM32H723 + FDCAN1）。

| 文件 | 作用 |
| --- | --- |
| [`demo/qzb_canfd.py`](demo/qzb_canfd.py) | 公共层：后端库定位、终端电阻、CAN FD 位定时、MIT 帧打包与反馈解析 |
| [`demo/quanzhibo_canfd.py`](demo/quanzhibo_canfd.py) | 原始报文示例：发 8 关节使能帧（`0x10`，64 字节），带总线错误统计 |
| [`demo/qzb_sine.py`](demo/qzb_sine.py) | 速度正弦跟随：同一帧里驱动多台电机（默认 ID 1/2/3） |

---

## 1. 硬件与通信参数

```
PC ──USB──> USB2CANFD ──CAN_H/CAN_L──> 泉智博驱动器 ──> 电机
```

| 项 | 值 |
| --- | --- |
| 帧类型 | 标准帧（11 位 ID），CAN FD，**BRS 开启** |
| 仲裁域 | 1 Mbps |
| 数据域 | 5 Mbps |
| 终端电阻 | 总线两端 120Ω；USB-CAN 端支持软件开关，脚本自动打开 |
| 注意 | 该电机 CAN_L / CAN_H 与 DM 系列 **线序相反**，接线前确认 |

### 位定时

说明书按 170 MHz 主频给出，而这台 USB-CAN 设备的 CAN 时钟是 **80 MHz**，无法照抄分频。
`qzb_canfd.configure_canfd()` 会反推设备时钟并取最接近的划分：

| | 说明书（170 MHz） | 本设备（80 MHz） |
| --- | --- | --- |
| 仲裁域 | brp 10, seg1 13, seg2 3 → 17 tq, SP 82.3% | brp 2, seg1 32, seg2 7 → 40 tq, **SP 82.5%** |
| 数据域 | brp 1, seg1 25, seg2 8 → 34 tq, SP 76.47% | brp 1, seg1 11, seg2 4 → 16 tq, **SP 75.0%** |

两个硬性要求（实测）：

- **必须是 ISO CAN FD**（`non_iso = 0`），非 ISO 模式下电机不 ACK，数据段报 `dlec=4`；
- 5 Mbps 数据段必须打开 **TDC**（`tdc_offset = 0` 为自动）。

---

## 2. 协议

### 2.1 命令帧

| ID | 功能 | 数据 |
| --- | --- | --- |
| `0x10` | 8 关节使能 | `FF FF FF FF FF FF FF FC` × 8（64 字节） |
| `0x10` | 8 关节失能 | `FF FF FF FF FF FF FF FD` × 8（64 字节） |
| `0x20` | MIT 控制 | 8 关节 × 8 字节，按槽位填入 |

### 2.2 MIT 单关节 8 字节（大端位域）

| 字节 | 字段 | 映射 |
| --- | --- | --- |
| Byte0-1 | θ_ref（16 bit） | `0x0000 → -12.5 rad`，`0xFFFF → +12.5 rad` |
| Byte2 + Byte3 高 4 位 | V_ref（12 bit） | `0x000 → -65 rad/s`，`0xFFF → +65 rad/s` |
| Byte3 低 4 位 + Byte4 | Kp（12 bit） | 0 ~ 500 |
| Byte5 + Byte6 高 4 位 | Kd（12 bit） | 0 ~ 5 |
| Byte6 低 4 位 + Byte7 | T_ref（12 bit） | `0x000 → -30 N·m`，`0xFFF → +30 N·m` |

速度模式：`Kp = 0`，`Kd > 0`，只有 `V_ref` 起控制作用，位置与前馈力矩给 0。

### 2.3 反馈帧

每台电机单独发一帧，**ID 即电机编号**，8 字节：

| 字节 | 内容 |
| --- | --- |
| Byte0-1 | 角度 θ（16 bit，映射同 θ_ref） |
| Byte2 + Byte3 高 4 位 | 速度 V（12 bit） |
| Byte3 低 4 位 + Byte4 | 扭矩 T（12 bit） |
| Byte5 | 线圈温度（值 - 40 ℃） |
| Byte6-7 | 16 位错误码 |

错误码位：

| 位 | 含义 | 位 | 含义 |
| --- | --- | --- | --- |
| bit0 | 使能指示（1 = 已使能） | bit8 | 堵转 |
| bit1 | 母线过压 | bit10 | 驱动板过温 |
| bit2 | 相电流过流 | bit11 | 母线欠压 |
| bit3 | 绕组过热 | bit12 | 位置超限 |
| bit4 | 超速 | bit13 | CAN 超时 |

### 2.4 槽位与反馈 ID

`0x20` 的 64 字节分成 8 个槽位，每台电机只读自己那 8 字节：

```
slot 0 | slot 1 | slot 2 | slot 3 | ... | slot 7
 0-7      8-15     16-23    24-31         56-63
```

**默认对应关系是 `槽位 = 反馈 ID - 1`**。实测反馈 ID 3 的电机在槽位 2。
换电机后如果不动，用 `--slots` 显式指定，或从 0 到 7 逐个试。

---

## 3. 安装

```bash
pip install pack/dmcan_sdk-2.1.1-py3-none-any.whl
```

后端库由脚本自动查找（`DMCAN_LIBRARY_PATH` 优先），规则与本目录 [`README.md`](README.md) 相同。

---

## 4. 快速开始

以下命令都在 `demo/` 目录下执行：

```bash
cd demo
```

### 4.1 原始报文示例

```bash
python3 quanzhibo_canfd.py --count 1     # 发一帧 0x10 使能帧
python3 quanzhibo_canfd.py               # 连发 10 帧
```

会打印位定时、实际发送的数据和总线错误统计。注意它**只发使能帧**，发完请确保后续有
MIT 控制帧或失能帧，否则电机会因 CAN 超时进入保护（见第 6 节）。

### 4.2 速度正弦跟随

```bash
python3 qzb_sine.py                              # ID 1/2/3 同时 ±10 rad/s、0.2 Hz、10 秒
python3 qzb_sine.py --amplitude 3 --duration 4
python3 qzb_sine.py --ids 1,2 --amplitude 5
python3 qzb_sine.py --ids 3 --slots 2            # 显式指定槽位
python3 qzb_sine.py --kd 3 --rate 500            # 阻尼更大、控制帧更快
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--ids` | `1,2,3` | 要驱动的电机反馈 ID 列表 |
| `--slots` | `ID-1` | 各电机的 MIT 槽位 |
| `--amplitude` | 10 | 速度正弦峰值 rad/s（量程 ±65） |
| `--freq` | 0.2 | 正弦频率 Hz |
| `--duration` | 10 | 运行时长 s |
| `--ramp` | 1.0 | 幅度渐入时间 s |
| `--kd` | 2.0 | 速度环阻尼（量程 0~5） |
| `--rate` | 500 | 控制帧频率 Hz |
| `--arb-sp` / `--data-sp` | 0.823 / 0.7647 | 采样点，取自说明书 |
| `--no-restore` | 关 | 结束后不恢复通道波特率 |

运行流程：使能广播 → 等待每台电机首帧反馈 → 按正弦下发速度 → 速度归零 → 失能 →
恢复通道配置。中途任一电机报故障（CAN 超时除外）会立即中止。

---

## 5. 实测记录

硬件：USB2CANFD_S + 泉智博驱动器 ×3（ID 1/2/3），1 Mbps / 5 Mbps。

| 项目 | 结果 |
| --- | --- |
| 反馈帧 ID | 1、2、3，对应 MIT 槽位 0、1、2 |
| 速度跟随（±3 rad/s、0.2 Hz、Kd=2） | ID1/2/3 RMS 误差均约 **3%**（0.08~0.09 rad/s） |
| 单台跑（±5 rad/s） | ID3 RMS 误差 0.10 rad/s（**2%**） |
| 位置反馈 | 多圈超出 ±12.5 rad 时读数会停在边界，速度反馈不受影响 |

---

## 6. 常见问题

**完全没有反馈 / 收不到 ACK**

1. 驱动器是否上电；
2. CAN_H / CAN_L 是否接反（与 DM 系列相反）；
3. 终端电阻：两端各 120Ω；
4. 是否用了 ISO CAN FD（`non_iso=0`）并开启 TDC；
5. 数据段采样点：0.75 / 0.80 都可能失败，按说明书 76.47%（本设备取 75.0%）。

**`err = 0x2000`（CAN 超时）、`enabled = False`**

电机被使能后如果一段时间收不到自己的 MIT 指令就会进超时保护，且**锁存**，重发使能帧
清不掉，需要给该电机断电重上电。用 `quanzhibo_canfd.py` 单独发使能帧时尤其容易触发，
测试完记得发失能帧。

**`enabled = True` 但速度不跟**

多半是槽位不对。看反馈 ID，再按 `槽位 = ID - 1` 填；不确定就从 0 到 7 逐个试。

**电机只是轻微响、不动**

`Kd` 太小或速度目标太低。速度环力矩大致与 `Kd × 速度误差` 成正比，先加大 `--kd`
（不超过 5）或提高 `--amplitude`。

**上电后第一帧就报错**

检查电机是否处于机械限位（bit12 位置超限）或母线电压（bit11），这两类需要先排除硬件原因。
