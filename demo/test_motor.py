"""
@file test_motor.py
@brief GO-M8010-6（经 CAN转485 模块）接线与协议自检：设/读 Kpos·Kspd，并打印所有上传帧。

默认**不会让电机转动**：
    1. 发控制模式 11 设置 Kpos/Kspd；
    2. 发控制模式 12 回读参数；
    3. 1 秒内打印收到的所有上传帧，用来确认模块 ID / 电机 ID / 波特率是否正确。

确认通信正常后，加 `--move` 才会发控制帧（模式 13，零位置低刚度保持），动作要自己盯紧。

用法：
    python3 test_motor.py                        # 模块 ID 3、电机 ID 0、通道 0
    python3 test_motor.py --module-id 2 --motor-id 1
    python3 test_motor.py --bitrate 500000       # 模块拨码第 4 位拨到 ON 时是 500k
    python3 test_motor.py --move                 # 再做一次小幅动作测试
"""

import argparse
import os
import pathlib
import sys
import time

# ------------------------------------------------------------------ 后端库定位
# 与 demo.py 同一套规则：先看本目录 dlls/，Linux 上再按架构找仓库里的预编译库
_DEMO_DIR = pathlib.Path(__file__).resolve().parent
_REPO_LIB_DIR = next((p / "C&C++" / "lib" / "v2.1.0"
                      for p in _DEMO_DIR.parents
                      if (p / "C&C++" / "lib" / "v2.1.0").is_dir()),
                     _DEMO_DIR.parents[1] / "C&C++" / "lib" / "v2.1.0")


def _prepare_backend_library() -> None:
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


_prepare_backend_library()

from dmcan import DmCanContext, dmcan_module_feature, dmcan_event_flag  # noqa: E402

from go_m8010_can import GoM8010Can  # noqa: E402


def enable_termination(device, channel: int) -> None:
    """
    @brief 打开通道的软件终端电阻（120Ω）；老硬件由板上物理开关决定。

    @param device 已打开的 `DmCanDevice`
    @param channel 通道号（必须已 `enable_channel`）
    """
    try:
        features = device.get_module_features()
    except Exception as exc:
        print(f"读不到模块能力位，跳过终端电阻设置：{exc}")
        return
    if not features & dmcan_module_feature.CAN_RES_SW:
        print("本模块不支持软件终端电阻（老硬件用板上开关），请确认物理开关已打开")
        return
    try:
        if device.get_can_res(channel):
            print(f"通道 {channel} 终端电阻已处于接通状态")
        else:
            device.enable_can_res(channel, True)
            print(f"已打开通道 {channel} 的软件终端电阻")
    except Exception as exc:
        print(f"终端电阻设置失败：{exc}")


def describe_upload(can_id: int, data: bytes) -> str:
    """
    @brief 把上传帧的 29 位 ID 拆成可读字段。

    @param can_id CAN ID
    @param data 数据区
    @return 一行描述
    """
    module = (can_id >> 27) & 0x3
    direction = (can_id >> 26) & 0x1
    content = (can_id >> 24) & 0x3
    low3 = (can_id >> 16) & 0xFF
    low2 = (can_id >> 8) & 0xFF
    low1 = can_id & 0xFF
    motor = low2 & 0xF
    status = (low2 >> 4) & 0x7
    if direction == 0:
        return (f"（这是下发回环）module={module} motor={motor} content={content} "
                f"mode={low3} status={status} data={data.hex()}")
    kind = {1: "电机响应", 2: "参数回读"}.get(content, f"content={content}")
    line = (f"{kind}：module={module} motor={motor} status={status} "
            f"低位3={low3} 低位1={low1}(温度/错误码)")
    if content == 1 and len(data) >= 8:
        pos = int.from_bytes(data[0:4], "little", signed=True)
        vel = int.from_bytes(data[4:6], "little", signed=True)
        tau = int.from_bytes(data[6:8], "little", signed=True)
        line += (f" | 位置={GoM8010Can.counts_to_position(pos):.3f} rad"
                 f" 速度={GoM8010Can.counts_to_velocity(vel):.3f} rad/s"
                 f" 力矩={GoM8010Can.counts_to_torque(tau):.3f} N·m")
    elif content == 2 and len(data) >= 4:
        kd = int.from_bytes(data[0:2], "little")
        kp = int.from_bytes(data[2:4], "little")
        line += f" | Kspd={kd}(κw={kd / 1280:.3f}) Kpos={kp}(κp={kp / 1280:.3f})"
    return line + f" raw={data.hex()}"


def main() -> int:
    """
    @brief 跑自检流程。

    @return 进程退出码（0 = 收到过上传帧）
    """
    parser = argparse.ArgumentParser(description="GO-M8010-6 CAN 自检")
    parser.add_argument("--channel", type=int, default=0, help="CAN 通道号（默认 0）")
    parser.add_argument("--module-id", type=int, default=3, help="CAN转485 模块 ID 0~3（默认 3）")
    parser.add_argument("--motor-id", type=int, default=0, help="电机 ID 0~14（默认 0）")
    parser.add_argument("--kp", type=float, default=0.1, help="刚度 κp（默认 0.1）")
    parser.add_argument("--kd", type=float, default=0.2, help="阻尼 κw（默认 0.2）")
    parser.add_argument("--bitrate", type=int, default=0,
                        help="把通道改成经典 CAN 的该波特率（0 = 用设备里保存的配置）")
    parser.add_argument("--move", action="store_true", help="再做一次零位置低刚度保持（电机会动）")
    args = parser.parse_args()

    received = []

    def on_frame(dev, event):
        frame = event.frame
        data = bytes(frame.payload[: frame.dlen])
        line = f"RX ch={event.channel} id=0x{frame.can_id:08X} ext={frame.ext} dlen={frame.dlen} {data.hex()}"
        print(line)
        print("   ->", describe_upload(frame.can_id, data))
        received.append((frame.can_id, data))

    with DmCanContext() as context:
        if context.find_devices() == 0:
            print("没有找到设备，检查 USB 连接")
            return 2
        device = context.get_device(0)
        device.open()
        try:
            if args.bitrate:
                info = device.get_channel_baudrate(args.channel)
                info.canfd = False
                info.can_baudrate = args.bitrate
                device.set_channel_baudrate(args.channel, info)
                print(f"通道 {args.channel} 已设为经典 CAN {args.bitrate} bps")

            baud = device.get_channel_baudrate(args.channel)
            print(f"通道 {args.channel}：canfd={bool(baud.canfd)} 标称={baud.can_baudrate} "
                  f"数据段={baud.canfd_baudrate}")
            if baud.canfd:
                print("警告：模块只支持经典 CAN，请先关掉 CAN FD（demo 里是 canfd=False）")

            device.enable_channel(args.channel, True)
            enable_termination(device, args.channel)

            subscription = device.subscribe(dmcan_event_flag.RX_FRAME, on_frame,
                                            channel_mask=1 << args.channel)
            try:
                motor = GoM8010Can(device, args.channel, args.module_id, args.motor_id)
                print(f"下发：模块 {args.module_id} / 电机 {args.motor_id}，"
                      f"设 Kpos=κp*1280={round(args.kp * 1280)}, Kspd=κw*1280={round(args.kd * 1280)}")
                motor.set_gains(args.kp, args.kd)
                motor.read_gains()
                time.sleep(1.0)

                if args.move:
                    print("动作测试：模式 13，零位置低刚度保持 1 秒……")
                    motor.send(position=0.0, velocity=0.0, torque=0.0, quiet=True)
                    time.sleep(1.0)
                    motor.lock()
                    time.sleep(0.3)
            finally:
                subscription.cancel()
                device.disable_channel(args.channel)
        finally:
            device.close()

    if received:
        print(f"\n共收到 {len(received)} 帧，通信正常。")
        return 0
    print("\n没有收到任何上传帧。排查顺序：")
    print("  1. 模块拨码 ID（前两位）是否等于 --module-id；默认出厂是 3；")
    print("  2. 模块拨码第 4 位：ON=500k、OFF=1M，与 --bitrate / 通道配置是否一致；")
    print("  3. 电机 ID 是否等于 --motor-id；")
    print("  4. 总线是否 120Ω 终端、CAN_H/CAN_L 是否接反。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
