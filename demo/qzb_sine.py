"""
@file qzb_sine.py
@brief 泉智博（QZB）电机速度正弦跟随：通过 USB2CANFD 以 CAN FD 发 MIT 控制帧，
       同一帧里可同时驱动多台电机（默认 ID 1、2、3）。

协议与位定时说明见 `qzb_canfd.py`；本文件只负责轨迹与多电机调度。

槽位与反馈 ID 的对应：默认 **槽位 = 反馈 ID - 1**（实测反馈 ID 3 的电机在槽位 2）。
换电机如果不动，用 `--slots` 显式指定，或从 0 到 7 逐个试。

速度模式：`kp = 0`，`kd > 0`，`V_ref = A·sin(2πft)`，位置与力矩给 0。

用法：
    python3 qzb_sine.py                              # ID 1/2/3 同时 ±10 rad/s，0.2 Hz，10 秒
    python3 qzb_sine.py --ids 1,2 --amplitude 5
    python3 qzb_sine.py --ids 3 --slots 2            # 显式指定槽位
    python3 qzb_sine.py --kd 3 --rate 500
"""

import argparse
import math
import time

from qzb_canfd import (DISABLE_PATTERN, ENABLE_PATTERN, ERROR_TEXT, build_mit_frame,
                       configure_canfd, enable_termination, unpack_feedback)

from dmcan import DmCanContext, dmcan_event_flag


class MotorFeedback:
    """@brief 单个电机的最新反馈；回调里只做赋值，避免占用通知线程。"""

    __slots__ = ("vel", "pos", "tor", "temp", "err", "enabled", "count")

    def __init__(self) -> None:
        self.vel = None
        self.pos = None
        self.tor = None
        self.temp = None
        self.err = 0
        self.enabled = False
        self.count = 0

    def update(self, info: dict) -> None:
        """@brief 收下一帧反馈。"""
        self.vel = info["vel"]
        self.pos = info["pos"]
        self.tor = info["tor"]
        self.temp = info["temp"]
        self.err = info["err"]
        self.enabled = info["enabled"]
        self.count += 1


def _parse_int_list(text: str) -> list:
    """
    @brief 解析逗号分隔的整数列表（支持 0x 前缀）。

    @param text 形如 "1,2,3"
    @return 整数列表
    """
    values = [int(item, 0) for item in text.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("列表不能为空")
    return values


def main() -> int:
    """
    @brief 多电机速度正弦跟随主流程。

    @return 进程退出码（0 = 正常完成）
    """
    parser = argparse.ArgumentParser(description="泉智博电机速度正弦跟随（CAN FD MIT 模式）")
    parser.add_argument("--channel", type=int, default=0, help="CAN 通道号（默认 0）")
    parser.add_argument("--ids", type=_parse_int_list, default=[1, 2, 3],
                        help="要驱动的电机反馈 ID 列表，逗号分隔（默认 1,2,3）")
    parser.add_argument("--slots", type=_parse_int_list, default=None,
                        help="显式指定各电机的 MIT 槽位（默认 = ID - 1）")
    parser.add_argument("--amplitude", type=float, default=10.0,
                        help="速度正弦峰值 rad/s（默认 10，量程 ±65）")
    parser.add_argument("--freq", type=float, default=0.2, help="正弦频率 Hz（默认 0.2）")
    parser.add_argument("--duration", type=float, default=10.0, help="运行时长 s（默认 10）")
    parser.add_argument("--ramp", type=float, default=1.0, help="幅度渐入时间 s（默认 1）")
    parser.add_argument("--kd", type=float, default=2.0, help="速度环阻尼 Kd（默认 2.0，量程 0~5）")
    parser.add_argument("--rate", type=float, default=500.0, help="控制帧频率 Hz（默认 500）")
    parser.add_argument("--arb-bitrate", type=int, default=1_000_000)
    parser.add_argument("--data-bitrate", type=int, default=5_000_000)
    parser.add_argument("--arb-sp", type=float, default=0.823, help="仲裁域采样点（说明书 82.3%）")
    parser.add_argument("--data-sp", type=float, default=0.7647, help="数据域采样点（说明书 76.47%）")
    parser.add_argument("--no-restore", action="store_true", help="结束后不恢复通道波特率")
    args = parser.parse_args()

    if args.slots is None:
        slots = [motor_id - 1 for motor_id in args.ids]
    else:
        slots = args.slots
    if len(slots) != len(args.ids):
        print("--slots 的个数必须与 --ids 一致")
        return 2
    if any(not 0 <= slot <= 7 for slot in slots):
        print("槽位必须在 0~7")
        return 2
    if not 0 < args.freq < args.rate / 5:
        print("频率过高：控制帧至少是正弦的 5 倍采样")
        return 2

    feedback = {motor_id: MotorFeedback() for motor_id in args.ids}
    slot_of = dict(zip(args.ids, slots))

    def on_frame(dev, event):
        frame = event.frame
        state = feedback.get(frame.can_id)  # 反馈帧 ID 即电机 ID
        if state is not None:
            data = bytes(frame.payload[: frame.dlen])
            if len(data) >= 8:
                state.update(unpack_feedback(data))

    period = 1.0 / args.rate
    omega = 2.0 * math.pi * args.freq
    original = None

    with DmCanContext() as context:
        if context.find_devices() == 0:
            print("没有找到设备，检查 USB 连接")
            return 2
        device = context.get_device(0)
        device.open()
        try:
            device.enable_channel(args.channel, True)
            enable_termination(device, args.channel)

            original = device.get_channel_baudrate(args.channel)
            if original.can_baudrate == 0 or (original.canfd and original.canfd_baudrate == 0):
                original.canfd = False
                original.can_baudrate = 1_000_000
                original.canfd_baudrate = 0
                original.can_sp = 0.75
            device.disable_channel(args.channel)
            configure_canfd(device, args.channel, args.arb_bitrate, args.data_bitrate,
                            args.arb_sp, args.data_sp)
            device.enable_channel(args.channel, True)

            subscription = device.subscribe(dmcan_event_flag.RX_FRAME, on_frame,
                                            channel_mask=1 << args.channel)
            try:
                # 1) 使能
                for _ in range(10):
                    device.send_can(args.channel, 0x10, ENABLE_PATTERN * 8,
                                    canfd=True, brs=True)
                    time.sleep(0.02)
                mapping = "，".join(f"ID {mid}→槽位 {slot_of[mid]}" for mid in args.ids)
                print(f"已发送使能帧；{mapping}；Kd={args.kd}")

                # 等各电机第一帧反馈
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    if all(state.count > 0 for state in feedback.values()):
                        break
                    time.sleep(0.01)
                silent = [mid for mid, state in feedback.items() if state.count == 0]
                for mid, state in feedback.items():
                    if state.count:
                        print(f"ID {mid} 在线：位置 {state.pos:+.2f} rad，"
                              f"速度 {state.vel:+.2f} rad/s，温度 {state.temp:.0f}℃，"
                              f"使能={state.enabled}")
                if silent:
                    print(f"ID {silent} 没有反馈；仍会继续，可用 --slots 调整槽位。")

                # 2) 速度正弦
                t0 = time.monotonic()
                next_tick = t0
                last_print = 0.0
                errors = {mid: [] for mid in args.ids}
                while True:
                    t = time.monotonic() - t0
                    if t >= args.duration:
                        break
                    # bit0 是使能指示，bit13 是 CAN 超时（持续发帧就会清掉），都不算致命
                    fault = {mid: state.err for mid, state in feedback.items()
                             if state.err & ~((1 << 0) | (1 << 13)) and t > 0.5}
                    if fault:
                        detail = "；".join(
                            f"ID {mid} 0x{err:04X}"
                            f"（{'、'.join(text for bit, text in ERROR_TEXT.items() if err & bit)}）"
                            for mid, err in fault.items())
                        print(f"\n电机报错：{detail}，已中止")
                        break

                    ramp = 1.0 if args.ramp <= 0 else min(1.0, t / args.ramp)
                    target = args.amplitude * ramp * math.sin(omega * t)
                    device.send_can(args.channel, 0x20,
                                    build_mit_frame({slot: target for slot in slots}, args.kd),
                                    canfd=True, brs=True)

                    if t >= args.ramp:
                        for mid, state in feedback.items():
                            if state.vel is not None:
                                errors[mid].append(target - state.vel)

                    if t - last_print >= 0.5:
                        last_print = t
                        parts = [f"目标={target:+6.2f}"]
                        for mid, state in feedback.items():
                            actual = "  n/a" if state.vel is None else f"{state.vel:+6.2f}"
                            parts.append(f"ID{mid}={actual}")
                        print(f"t={t:5.2f}s  " + "  ".join(parts) + " rad/s")

                    next_tick += period
                    sleep = next_tick - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_tick = time.monotonic()

                for mid, samples in errors.items():
                    if not samples:
                        continue
                    rms = math.sqrt(sum(e * e for e in samples) / len(samples))
                    print(f"ID {mid} 跟踪误差：RMS={rms:.2f} rad/s，"
                          f"最大={max(abs(e) for e in samples):.2f} rad/s"
                          f"（{rms / args.amplitude * 100:.0f}%）")

                # 3) 速度归零，再失能
                for _ in range(100):
                    device.send_can(args.channel, 0x20,
                                    build_mit_frame({slot: 0.0 for slot in slots}, args.kd),
                                    canfd=True, brs=True)
                    time.sleep(0.005)
                print("速度已归零。")
            finally:
                subscription.cancel()
                try:
                    for _ in range(5):
                        device.send_can(args.channel, 0x10, DISABLE_PATTERN * 8,
                                        canfd=True, brs=True)
                        time.sleep(0.02)
                    print("已发送失能帧。")
                except Exception as exc:
                    print(f"失能帧发送失败（忽略）：{exc}")
        except KeyboardInterrupt:
            print("\n中断，已发送失能帧。")
        finally:
            if original is not None and not args.no_restore:
                try:
                    device.disable_channel(args.channel)
                    device.set_channel_baudrate(args.channel, original)
                    time.sleep(0.2)
                    device.enable_channel(args.channel, True)
                    print("已恢复通道原来的波特率配置。")
                except Exception as exc:
                    print(f"恢复原配置失败（忽略）：{exc}")
            device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
