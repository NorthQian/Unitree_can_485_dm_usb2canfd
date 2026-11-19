"""
@file go_m8010_can.py
@brief 宇树 GO-M8010-6 电机经「CAN转485 模块」的发送封装（29 位扩展帧，经典 CAN）。

协议出处：https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/CAN_to_485

CAN ID（29 位）位域：

    bit 28-27  模块 ID（拨码开关设定，0~3）
    bit 26     方向：0 = 下发，1 = 上传
    bit 25-24  内容：0 = 电机控制，1 = 电机响应
    bit 23-16  文档称「低位3」= 控制模式 10 / 11 / 12 / 13
    bit 15-8   文档称「低位2」= 电机 ID(4bit) | 工作状态(3bit) | 保留(1bit)
    bit 7-0    文档称「低位1」= 下发时预留（上传时是温度 / 报错标志）

数据区 8 字节（多字节为小端，低位字节在前）：

    模式 10 / 13：位置 int32 @0-3、速度 int16 @4-5、力矩 int16 @6-7
    模式 11     ：阻尼 Kspd uint16 @0-1、刚度 Kpos uint16 @2-3，其余补 0
    模式 12     ：数据区无意义，发全 0；回复走「上传·返回内容 2」

定点编码（模块对控制参数是透传，换算同 485 协议）：

    位置 counts = rad / (2π) * 32768      （int32，±411774 rad）
    速度 counts = (rad/s) / (2π) * 256    （int16，±804.0 rad/s）
    力矩 counts = N·m * 256               （int16，±127.99 N·m）
    Kpos / Kspd = κ * 1280                （uint16，0 ~ 25.599）

使用顺序：上电后先 `set_gains()`（模式 11）设好 Kpos/Kspd（模块断电即丢），
再反复 `send()`。模式 10 每次控制都回一帧数据，高频会占满总线；
模式 13 默认不回、仅报错时回，需要数据时用 `read_gains()` 之类的问答。
"""

from __future__ import annotations

import math
import struct

__all__ = ["GoM8010Can"]

_TWO_PI = 2.0 * math.pi

# 定点换算系数
_POS_SCALE = 32768.0 / _TWO_PI  # rad -> counts
_SPD_SCALE = 256.0 / _TWO_PI    # rad/s -> counts
_TAU_SCALE = 256.0              # N·m -> counts
_GAIN_SCALE = 1280.0            # κ -> counts


def _counts(name: str, value: float, scale: float, limit: float, lo: int, hi: int) -> int:
    """
    @brief 物理量按比例换成正整数并校验范围。

    @param name 物理量名（报错用）
    @param value 物理量数值
    @param scale 倍率
    @param limit 允许的物理量绝对值上限
    @param lo 定点值下限
    @param hi 定点值上限
    @return 定点整数值
    """
    if not math.isfinite(value) or abs(value) > limit:
        raise ValueError(f"{name}={value} 超出允许范围 ±{limit}")
    raw = int(round(value * scale))
    return max(lo, min(hi, raw))


class GoM8010Can:
    """
    @brief 一台电机的 CAN 发送封装：绑好通道 / 模块 ID / 电机 ID 后直接发指令。

    @note 只负责组帧与下发，不解析上传帧；CAN 侧的 CRC 由转换模块处理，上位机不用算。
    """

    CMD_CONTROL = 10
    """控制模式 10：下发控制参数，且每次都会回一帧（mode1）。"""
    CMD_SET_GAINS = 11
    """控制模式 11：设置 Kpos / Kspd。"""
    CMD_READ_GAINS = 12
    """控制模式 12：回读 Kpos / Kspd。"""
    CMD_CONTROL_QUIET = 13
    """控制模式 13：下发控制参数，默认不回（mode2）。"""

    STATUS_LOCK = 0
    """工作状态：锁定。"""
    STATUS_FOC = 1
    """工作状态：FOC 闭环。"""
    STATUS_CALIBRATE = 2
    """工作状态：编码器校准（等待 5s，期间不得发任何数据包）。"""

    LIMIT_TORQUE = 127.99
    LIMIT_SPEED = 804.0
    LIMIT_POSITION = 411774.0
    LIMIT_GAIN = 25.599

    def __init__(self, device, channel: int, module_id: int, motor_id: int = 0) -> None:
        """
        @param device 已打开的 `DmCanDevice`
        @param channel 该电机所在的 CAN 通道号
        @param module_id CAN转485 模块 ID（0~3，拨码开关前两位）
        @param motor_id 电机 ID（0~14；15 表示广播，这里不允许）
        """
        if not 0 <= module_id <= 3:
            raise ValueError(f"module_id={module_id} 超出 0~3")
        if not 0 <= motor_id <= 14:
            raise ValueError(f"motor_id={motor_id} 超出 0~14（15 是广播，不支持）")
        self.device = device
        self.channel = channel
        self.module_id = module_id
        self.motor_id = motor_id

    # ------------------------------------------------------------------ 组帧

    def make_id(self, cmd: int, status: int = 0, direction: int = 0, content: int = 0) -> int:
        """
        @brief 拼 29 位扩展帧 ID。

        @param cmd 低位3：控制模式
        @param status 低位2 里的工作状态（0 锁定 / 1 FOC / 2 校准）
        @param direction 0 = 下发，1 = 上传
        @param content 0 = 电机控制，1 = 电机响应
        @return 29 位 CAN ID
        """
        return (((self.module_id & 0x3) << 27)
                | ((direction & 0x1) << 26)
                | ((content & 0x3) << 24)
                | ((cmd & 0xFF) << 16)
                | ((self.motor_id & 0xF) << 8)
                | ((status & 0x7) << 12))

    def _send(self, cmd: int, status: int, data: bytes) -> None:
        """
        @brief 以扩展帧发出 8 字节数据区。

        @param cmd 控制模式
        @param status 工作状态
        @param data 恰好 8 字节
        """
        assert len(data) == 8, "数据区必须是 8 字节"
        self.device.send_can(self.channel, self.make_id(cmd, status), data,
                             canfd=False, ext=True, rtr=False, brs=False)

    # ------------------------------------------------------------------ 发送

    def set_gains(self, kp: float, kd: float) -> None:
        """
        @brief 控制模式 11：设置刚度 Kpos 与阻尼 Kspd（模块断电丢失，重新上电要重发）。

        @param kp 刚度系数 κp，范围 0 ~ 25.599
        @param kd 阻尼系数 κw，范围 0 ~ 25.599
        """
        if not 0.0 <= kp <= self.LIMIT_GAIN:
            raise ValueError(f"kp={kp} 超出 0~{self.LIMIT_GAIN}")
        if not 0.0 <= kd <= self.LIMIT_GAIN:
            raise ValueError(f"kd={kd} 超出 0~{self.LIMIT_GAIN}")
        # 文档：低位2 在模式 11 下「ID、无意义、无意义」，只保留电机 ID
        data = struct.pack("<HH4x", int(round(kd * _GAIN_SCALE)), int(round(kp * _GAIN_SCALE)))
        self._send(self.CMD_SET_GAINS, 0, data)

    def send(self, position: float = 0.0, velocity: float = 0.0, torque: float = 0.0,
             status: int = STATUS_FOC, quiet: bool = False) -> None:
        """
        @brief 控制模式 10 / 13：下发期望位置、速度、力矩。

        @param position 期望位置，单位 rad（多圈累加），±411774
        @param velocity 期望速度，单位 rad/s，±804.0
        @param torque 期望力矩（前馈），单位 N·m，±127.99
        @param status 工作状态：0 锁定 / 1 FOC 闭环 / 2 编码器校准
        @param quiet True 用模式 13（不回帧），False 用模式 10（每帧回数据）
        @note 各项均为电机（转子）侧数值；GO-M8010-6 有 6.33:1 减速比，要按关节端
              给数需自行乘除该减速比。
        """
        data = struct.pack("<ihh",
                           _counts("position", position, _POS_SCALE, self.LIMIT_POSITION,
                                   -(2 ** 31), 2 ** 31 - 1),
                           _counts("velocity", velocity, _SPD_SCALE, self.LIMIT_SPEED,
                                   -(2 ** 15), 2 ** 15 - 1),
                           _counts("torque", torque, _TAU_SCALE, self.LIMIT_TORQUE,
                                   -(2 ** 15), 2 ** 15 - 1))
        self._send(self.CMD_CONTROL_QUIET if quiet else self.CMD_CONTROL, status, data)

    def read_gains(self) -> None:
        """
        @brief 控制模式 12：回读 Kpos / Kspd（回复为「上传·返回内容 2」）。
        """
        self._send(self.CMD_READ_GAINS, 0, bytes(8))

    def lock(self) -> None:
        """@brief 锁定电机：状态 0、控制量全零。"""
        self.send(0.0, 0.0, 0.0, status=self.STATUS_LOCK)

    # ------------------------------------------------------------ 上传帧解析

    def parse_upload(self, can_id: int, data: bytes) -> dict | None:
        """
        @brief 解析模块上传帧。

        @param can_id 收到的 CAN ID
        @param data 数据区（前 8 字节有效）
        @return 不是本模块/本电机的上传帧时返回 None；否则返回字典：
                `kind`（"motor" / "gains" / "error"）、其余字段按 `kind` 不同而不同：
                - motor：`position` / `velocity` / `torque`（转子侧 rad、rad/s、N·m）、
                  `temperature`（℃）、`pressure`（0~255）
                - error：`error`（错误代号）、`temperature` 为 -128
                - gains：`kp` / `kd`（κp、κw 的物理值）
        """
        if not ((can_id >> 26) & 0x1):
            return None  # 方向位为 0：下发回环，不是上传
        if ((can_id >> 27) & 0x3) != self.module_id:
            return None
        low2 = (can_id >> 8) & 0xFF
        if (low2 & 0xF) != self.motor_id:
            return None
        content = (can_id >> 24) & 0x3
        low3 = (can_id >> 16) & 0xFF
        low1 = can_id & 0xFF
        if content == 2:  # 模式 12 的参数回读
            if len(data) < 4:
                return None
            kd = int.from_bytes(data[0:2], "little")
            kp = int.from_bytes(data[2:4], "little")
            return {"kind": "gains", "kp": kp / _GAIN_SCALE, "kd": kd / _GAIN_SCALE}
        if content != 1:
            return {"kind": "unknown", "content": content, "low3": low3, "low1": low1}
        if len(data) < 8:
            return None
        temperature = low1 - 256 if low1 > 127 else low1
        if temperature == -128:  # 报文约定：-128 表示报错，此时低位3 是错误代号
            return {"kind": "error", "error": low3, "temperature": temperature}
        return {
            "kind": "motor",
            "position": self.counts_to_position(int.from_bytes(data[0:4], "little", signed=True)),
            "velocity": self.counts_to_velocity(int.from_bytes(data[4:6], "little", signed=True)),
            "torque": self.counts_to_torque(int.from_bytes(data[6:8], "little", signed=True)),
            "temperature": temperature,
            "pressure": low3,
        }

    # ------------------------------------------------------------ 定点换算

    @staticmethod
    def position_to_counts(rad: float) -> int:
        """@brief 位置 rad -> counts（参考用，`send()` 内部已换算）。"""
        return _counts("position", rad, _POS_SCALE, GoM8010Can.LIMIT_POSITION,
                       -(2 ** 31), 2 ** 31 - 1)

    @staticmethod
    def velocity_to_counts(rad_s: float) -> int:
        """@brief 速度 rad/s -> counts。"""
        return _counts("velocity", rad_s, _SPD_SCALE, GoM8010Can.LIMIT_SPEED,
                       -(2 ** 15), 2 ** 15 - 1)

    @staticmethod
    def torque_to_counts(nm: float) -> int:
        """@brief 力矩 N·m -> counts。"""
        return _counts("torque", nm, _TAU_SCALE, GoM8010Can.LIMIT_TORQUE,
                       -(2 ** 15), 2 ** 15 - 1)

    @staticmethod
    def counts_to_position(raw: int) -> float:
        """@brief 位置 counts -> rad。"""
        return raw / _POS_SCALE

    @staticmethod
    def counts_to_velocity(raw: int) -> float:
        """@brief 速度 counts -> rad/s。"""
        return raw / _SPD_SCALE

    @staticmethod
    def counts_to_torque(raw: int) -> float:
        """@brief 力矩 counts -> N·m。"""
        return raw / _TAU_SCALE
