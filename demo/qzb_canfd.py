"""
@file qzb_canfd.py
@brief 泉智博（QZB）电机 CAN FD 公共层：后端库定位、终端电阻、位定时、MIT 帧打包与反馈解析。

被 `quanzhibo_canfd.py`（单帧发送示例）和 `qzb_sine.py`（速度正弦跟随）共用。

协议来源：`Noetix_motorevo_mc02` 工程（STM32H723 + FDCAN1）

    ID 0x10，DLC 64：8 关节使能，数据 `FF FF FF FF FF FF FF FC` × 8
    ID 0x10，DLC 64：8 关节失能，数据 `FF FF FF FF FF FF FF FD` × 8
    ID 0x20，DLC 64：MIT 控制，8 关节 × 8 字节，槽位 0~7 对应关节 1~8

MIT 单关节 8 字节（大端位域）：

    Byte0-1        θ_ref  int16   0x0000→-12.5 rad，0xFFFF→+12.5 rad
    Byte2 + B3高4  V_ref  12bit   0x000→-65 rad/s，0xFFF→+65 rad/s
    B3低4 + Byte4  Kp     12bit   0→500
    Byte5 + B6高4  Kd     12bit   0→5
    B6低4 + Byte7  T_ref  12bit   0x000→-30 N·m，0xFFF→+30 N·m

反馈：每个电机单独一帧，ID 即电机编号，8 字节：
θ(16bit) / V(12bit) / T(12bit) / 温度(Byte5, 值-40℃) / 错误码(Byte6-7)

CAN FD 位定时：说明书按 170 MHz 主频给出（仲裁 82.3%、数据 76.47%）。本类设备 CAN 时钟是
80 MHz，`configure_canfd()` 会反推时钟并取最接近的分频（仲裁 82.5%、数据 75.0%），
必须用 **ISO CAN FD**（non_iso=0）并打开 TDC，否则电机不 ACK。
"""

from __future__ import annotations

import os
import pathlib
import sys
import time

_DEMO_DIR = pathlib.Path(__file__).resolve().parent
_REPO_LIB_DIR = next((p / "C&C++" / "lib" / "v2.1.0"
                      for p in _DEMO_DIR.parents
                      if (p / "C&C++" / "lib" / "v2.1.0").is_dir()),
                     _DEMO_DIR.parents[1] / "C&C++" / "lib" / "v2.1.0")


def prepare_backend_library() -> None:
    """@brief 未被环境变量覆盖时，按平台挑一个存在的后端库路径告诉 SDK。"""
    if "DMCAN_LIBRARY_PATH" in os.environ:
        return
    if sys.platform == "win32":
        candidates = [_DEMO_DIR / "dlls" / "dmcan.dll", _DEMO_DIR / "dlls" / "libdmcan.dll"]
    elif sys.platform == "darwin":
        candidates = [_DEMO_DIR / "dlls" / "libdmcan.dylib"]
    else:
        import platform
        arch = {"x86_64": "x86_64", "amd64": "x86_64",
                "aarch64": "arm64", "arm64": "arm64"}.get(platform.machine().lower())
        candidates = [_DEMO_DIR / "dlls" / "libdmcan.so"]
        if arch:
            candidates.append(_REPO_LIB_DIR / "linux" / arch / "libdmcan.so")
    for candidate in candidates:
        if candidate.is_file():
            os.environ["DMCAN_LIBRARY_PATH"] = str(candidate)
            return


prepare_backend_library()

from dmcan import dmcan_module_feature  # noqa: E402

ENABLE_PATTERN = b"\xff" * 7 + b"\xfc"
"""使能帧里每个关节的 8 字节。"""
DISABLE_PATTERN = b"\xff" * 7 + b"\xfd"
"""失能帧里每个关节的 8 字节。"""

# MIT 协议量程
THETA_MIN, THETA_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -65.0, 65.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
TOR_MIN, TOR_MAX = -30.0, 30.0

ERROR_TEXT = {
    1 << 0: "使能指示",
    1 << 1: "母线过压", 1 << 2: "相电流过流", 1 << 3: "绕组过热", 1 << 4: "超速",
    1 << 8: "堵转", 1 << 10: "驱动板过温", 1 << 11: "母线欠压",
    1 << 12: "位置超限", 1 << 13: "CAN 超时",
}


# ------------------------------------------------------------------ 通道配置

def enable_termination(device, channel: int) -> None:
    """
    @brief 打开通道的软件终端电阻（120Ω）；老硬件由板上物理开关决定。

    @param device 已打开的 `DmCanDevice`
    @param channel 通道号（必须已 `enable_channel`）
    """
    try:
        if device.get_module_features() & dmcan_module_feature.CAN_RES_SW:
            if device.get_can_res(channel):
                print(f"通道 {channel} 终端电阻已处于接通状态")
            else:
                device.enable_can_res(channel, True)
                print(f"已打开通道 {channel} 的软件终端电阻")
        else:
            print("本模块不支持软件终端电阻（老硬件用板上开关），请确认物理开关已打开")
    except Exception as exc:
        print(f"终端电阻设置失败：{exc}")


def _segments(clock_hz: int, bitrate: int, sample_point: float) -> tuple:
    """
    @brief 由 CAN 时钟与目标采样点算出一组 (brp, seg1, seg2, sjw)。

    @param clock_hz CAN 外设时钟
    @param bitrate 目标波特率
    @param sample_point 目标采样点（0~1）
    @return (brp, seg1, seg2, sjw)
    @note 每个位取 16~48 个时间份额：优先多取份额保证精度，时钟不够时退化为最大份额。
    """
    brp, quanta = 1, 0
    for candidate in range(1, 129):
        q = clock_hz / (candidate * bitrate)
        if q == int(q) and 16 <= q <= 48:
            brp, quanta = candidate, int(q)
            break
    if quanta == 0:  # 没有落在 16~48 的分频，退而求其次取最大的整数份额
        for candidate in (1, 2, 4, 8, 16, 32, 64, 128):
            q = clock_hz / (candidate * bitrate)
            if q == int(q) and 8 <= q <= 128:
                brp, quanta = candidate, int(q)
                break
    if quanta == 0:
        raise ValueError(f"CAN 时钟 {clock_hz} 无法分出 {bitrate} bps")
    seg1 = min(quanta - 2, max(1, int(round(sample_point * quanta)) - 1))
    seg2 = quanta - 1 - seg1
    return brp, seg1, seg2, min(4, seg2)


def configure_canfd(device, channel: int, arb_bitrate: int, data_bitrate: int,
                    arb_sp: float, data_sp: float) -> None:
    """
    @brief 把通道切成 CAN FD（仲裁域 / 数据域分别给波特率与采样点）。

    @param device 已打开的 `DmCanDevice`
    @param channel 通道号
    @param arb_bitrate 仲裁域波特率
    @param data_bitrate 数据域波特率
    @param arb_sp 仲裁域采样点（0~1）
    @param data_sp 数据域采样点（0~1）
    @note 目标采样点取自泉智博说明书：仲裁 82.3%、数据 76.47%。
    @note 说明书是 170 MHz 主频（数据段 34 tq），本设备 CAN 时钟是 80 MHz，只能取最接近的
          划分（仲裁 40 tq / 数据 16 tq），采样点误差 <2%。
    @note 必须是 **ISO CAN FD**（non_iso = 0）；实测非 ISO 模式电机不 ACK。
    @note 5 Mbps 数据段要开 TDC；`tdc_offset = 0` 表示自动。
    @note 固件每秒重载一次配置，写完要等一下才真正生效。
    """
    # 先按简单接口写一遍，库会算出设备当前的时钟/分频，用来反推 CAN 时钟
    info = device.get_channel_baudrate(channel)
    info.canfd = True
    info.non_iso = False
    info.retrans = True
    info.can_baudrate = arb_bitrate
    info.canfd_baudrate = data_bitrate
    info.can_sp = arb_sp
    info.canfd_sp = data_sp
    device.set_channel_baudrate(channel, info)

    probe = device.get_channel_baudrate_details(channel)
    clock = arb_bitrate * probe.nominal_brp * (1 + probe.nominal_seg1 + probe.nominal_seg2)
    nominal = _segments(clock, arb_bitrate, arb_sp)
    data = _segments(clock, data_bitrate, data_sp)

    config = probe
    config.enable_canfd = 1
    config.enable_tdc = 1
    config.non_iso = 0
    config.enable_retrans = 1
    config.nominal_brp, config.nominal_seg1, config.nominal_seg2, config.nominal_sjw = nominal
    config.data_brp, config.data_seg1, config.data_seg2, config.data_sjw = data
    config.tdc_offset = 0
    config.tdc_filter = 0
    device.set_channel_baudrate_details(channel, config)
    print(f"位定时（CAN 时钟 {clock / 1e6:.0f} MHz）："
          f"仲裁 brp={nominal[0]} seg1={nominal[1]} seg2={nominal[2]} "
          f"SP={(1 + nominal[1]) / (1 + nominal[1] + nominal[2]):.1%}；"
          f"数据 brp={data[0]} seg1={data[1]} seg2={data[2]} "
          f"SP={(1 + data[1]) / (1 + data[1] + data[2]):.1%}")
    time.sleep(1.1)


# ------------------------------------------------------------------ MIT 协议

def _raw(value: float, lo: float, hi: float, full_scale: int) -> int:
    """
    @brief 物理量 → MIT 原始整数值（先限幅）。

    @param value 物理量
    @param lo 量程下限
    @param hi 量程上限
    @param full_scale 满量程码值（65535 / 4095）
    @return 无符号整数
    """
    value = max(lo, min(hi, value))
    return int((value - lo) / (hi - lo) * full_scale)


def pack_joint(position: float, velocity: float, kp: float, kd: float,
               torque: float) -> bytes:
    """
    @brief 打包单个关节的 MIT 8 字节（与 MCU 端 mit_pack8 逐位一致）。

    @param position 目标位置 rad
    @param velocity 目标速度 rad/s
    @param kp 比例增益
    @param kd 微分增益
    @param torque 前馈力矩 N·m
    @return 8 字节
    """
    p = _raw(position, THETA_MIN, THETA_MAX, 65535)
    v = _raw(velocity, VEL_MIN, VEL_MAX, 4095)
    k = _raw(kp, KP_MIN, KP_MAX, 4095)
    d = _raw(kd, KD_MIN, KD_MAX, 4095)
    t = _raw(torque, TOR_MIN, TOR_MAX, 4095)
    return bytes((
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0x0F) << 4) | ((k >> 8) & 0x0F),
        k & 0xFF,
        (d >> 4) & 0xFF,
        ((d & 0x0F) << 4) | ((t >> 8) & 0x0F),
        t & 0xFF,
    ))


def build_mit_frame(velocity_by_slot: dict, kd: float) -> bytes:
    """
    @brief 组 8 关节 MIT 帧（64 字节），给定槽位填速度指令，其余槽位留空。

    @param velocity_by_slot 槽位号 → 目标速度 rad/s
    @param kd 所有受控槽位共用的微分增益
    @return 64 字节
    """
    frame = bytearray(64)
    for slot, velocity in velocity_by_slot.items():
        frame[slot * 8:(slot + 1) * 8] = pack_joint(0.0, velocity, 0.0, kd, 0.0)
    return bytes(frame)


def build_mit_frame_positions(positions_by_slot: dict, kp: float, kd: float,
                              velocity: float = 0.0) -> bytes:
    """
    @brief 组 8 关节 MIT 帧（64 字节），给定槽位填位置指令，其余槽位留空。

    @param positions_by_slot 槽位号 → 目标位置 rad
    @param kp 所有受控槽位共用的比例增益
    @param kd 所有受控槽位共用的微分增益
    @param velocity 速度前馈 rad/s（位置模式一般给 0）
    @return 64 字节
    """
    frame = bytearray(64)
    for slot, position in positions_by_slot.items():
        frame[slot * 8:(slot + 1) * 8] = pack_joint(position, velocity, kp, kd, 0.0)
    return bytes(frame)


def unpack_feedback(data: bytes) -> dict:
    """
    @brief 解析单个电机的 8 字节反馈帧。

    @param data 8 字节
    @return 含 pos / vel / tor / temp / err / enabled 的字典
    """
    p_int = (data[0] << 8) | data[1]
    v_int = (data[2] << 4) | ((data[3] >> 4) & 0x0F)
    t_int = ((data[3] & 0x0F) << 8) | data[4]
    err = (data[6] << 8) | data[7]
    return {
        "pos": THETA_MIN + p_int / 65535.0 * (THETA_MAX - THETA_MIN),
        "vel": VEL_MIN + v_int / 4095.0 * (VEL_MAX - VEL_MIN),
        "tor": TOR_MIN + t_int / 4095.0 * (TOR_MAX - TOR_MIN),
        "temp": data[5] - 40.0,
        "err": err,
        "enabled": bool(err & 1),
    }
