"""
@file sine_follow.py
@brief 让 GO-M8010-6（经 CAN转485 模块）的位置跟随正弦轨迹。

参考宇树官方位置模式参数：`q = 6.28*减速比, dq = 0, kp = 0.01, kd = 0.01, tau = 0`。
其中 `kp / kd / tau / dq` 是**转子侧**量，`q` 要乘减速比才是出轴角度。

流程：
    1. 先把增益设成 0 / 0.1，发几帧控制让模块持续上报，读出**当前转子位置**作为正弦中心；
    2. 改成目标 kp / kd；
    3. 按 `q_rotor = q0 + A*sin(2πft)*减速比` 以固定频率下发，出轴幅度 A，前 `--ramp` 秒渐入；
    4. 结束或 Ctrl-C 时先回零力矩再锁定。

用法：
    python3 sine_follow.py                                  # 出轴 ±0.5 rad、0.2 Hz、10 秒
    python3 sine_follow.py --amplitude 0.2 --duration 6
    python3 sine_follow.py --freq 0.5 --kp 0.05 --kd 0.2
"""

import argparse
import math
import os
import pathlib
import sys
import time

# ------------------------------------------------------------------ 后端库定位
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

_ERROR_TEXT = {0: "正常", 1: "过热", 2: "过流", 3: "过压", 4: "编码器故障",
               5: "母线欠压", 6: "绕组过热", 7: "未标定"}


def enable_termination(device, channel: int) -> None:
    """
    @brief 打开通道的软件终端电阻（120Ω）。

    @param device 已打开的 `DmCanDevice`
    @param channel 通道号（必须已 `enable_channel`）

    @note 只有能力位 `CAN_RES_SW` 为 1 的硬件支持；老硬件是板上的物理开关。
    @note 该状态不写 flash，设备重新枚举后要重新下发。
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
        print(f"终端电阻设置失败（通讯可能仍能跑，但长线易出错）：{exc}")


class LatestFeedback:
    """@brief 回调里只存最新一帧，避免在通知线程上做耗时的事。"""

    __slots__ = ("position", "velocity", "torque", "temperature", "error")

    def __init__(self) -> None:
        self.position = None
        self.velocity = None
        self.torque = None
        self.temperature = None
        self.error = None

    def update(self, info: dict) -> None:
        """@brief 收下一帧上传数据。"""
        if info["kind"] == "motor":
            self.position = info["position"]
            self.velocity = info["velocity"]
            self.torque = info["torque"]
            self.temperature = info["temperature"]
        elif info["kind"] == "error":
            self.error = info["error"]


def main() -> int:
    """
    @brief 跑正弦位置跟随。

    @return 进程退出码（0 = 正常跑完）
    """
    parser = argparse.ArgumentParser(description="GO-M8010-6 正弦位置跟随")
    parser.add_argument("--channel", type=int, default=0, help="CAN 通道号（默认 0）")
    parser.add_argument("--module-id", type=int, default=3, help="模块 ID 0~3（默认 3）")
    parser.add_argument("--motor-id", type=int, default=0, help="电机 ID 0~14（默认 0）")
    parser.add_argument("--gear-ratio", type=float, default=6.33,
                        help="减速比（GO-M8010-6 为 6.33），用于出轴角度与转子角度换算")
    parser.add_argument("--mode", choices=("velocity", "position"), default="velocity",
                        help="velocity = 让速度跟正弦（kp 必须为 0）；position = 让位置跟正弦")
    parser.add_argument("--amplitude", type=float, default=None,
                        help="出轴正弦峰值：velocity 模式 rad/s（默认 6.28，即 1 圈/秒）；"
                             "position 模式 rad（默认 0.5）")
    parser.add_argument("--freq", type=float, default=0.2, help="正弦频率，单位 Hz（默认 0.2）")
    parser.add_argument("--duration", type=float, default=10.0, help="运行时长，单位 s（默认 10）")
    parser.add_argument("--ramp", type=float, default=1.0, help="幅度渐入时间，单位 s（默认 1）")
    parser.add_argument("--kp", type=float, default=0.03,
                        help="刚度，与宇树 SDK 的 cmd.kp 同口径（速度模式强制为 0）")
    parser.add_argument("--kd", type=float, default=None,
                        help="阻尼，与宇树 SDK 的 cmd.kd 同口径（默认 velocity 0.01 / position 0.05）")
    parser.add_argument("--raw-gains", action="store_true",
                        help="不做减速比平方换算，直接把 --kp/--kd 当模块的 κp/κw 下发")
    parser.add_argument("--vel-ff", action="store_true",
                        help="把正弦的解析速度作为 dq 前馈（参考参数里 dq=0，默认关闭）")
    parser.add_argument("--rate", type=float, default=200.0, help="控制帧频率，单位 Hz（默认 200）")
    parser.add_argument("--no-termination", action="store_true",
                        help="不自动打开发送端的软件终端电阻（默认会打开）")
    args = parser.parse_args()

    if not 0 < args.freq < args.rate / 5:
        print("频率过高：控制帧至少要是正弦的 5 倍采样，检查 --freq / --rate")
        return 2

    feedback = LatestFeedback()

    def on_frame(dev, event):
        frame = event.frame
        info = motor.parse_upload(frame.can_id, bytes(frame.payload[: frame.dlen]))
        if info is not None:
            feedback.update(info)

    amplitude = args.amplitude if args.amplitude is not None else \
        (6.28 if args.mode == "velocity" else 0.5)
    period = 1.0 / args.rate
    motor = None

    with DmCanContext() as context:
        if context.find_devices() == 0:
            print("没有找到设备，检查 USB 连接")
            return 2
        device = context.get_device(0)
        device.open()
        try:
            baud = device.get_channel_baudrate(args.channel)
            print(f"通道 {args.channel}：canfd={bool(baud.canfd)} 标称={baud.can_baudrate}")
            if baud.canfd:
                print("警告：模块只支持经典 CAN，请先关掉 CAN FD")
                return 2
            device.enable_channel(args.channel, True)
            if not args.no_termination:
                enable_termination(device, args.channel)
            motor = GoM8010Can(device, args.channel, args.module_id, args.motor_id)

            subscription = device.subscribe(dmcan_event_flag.RX_FRAME, on_frame,
                                            channel_mask=1 << args.channel)
            try:
                # 1) 零刚度、轻阻尼，先读出当前转子位置（不受位置误差拉扯）
                motor.set_gains(0.0, 0.1)
                device_ready = False
                for _ in range(30):
                    motor.send(0.0, 0.0, 0.0)
                    time.sleep(0.01)
                    if feedback.position is not None:
                        device_ready = True
                        break
                if feedback.error is not None:
                    print(f"电机报错代号 {feedback.error}"
                          f"（{_ERROR_TEXT.get(feedback.error, '未知')}），"
                          "多数情况需要断电重新上电后才能恢复。")
                    return 1
                if not device_ready:
                    print("没有收到电机反馈，先跑 test_motor.py 排查接线 / ID / 波特率")
                    return 1

                q0 = feedback.position
                ratio = args.gear_ratio
                # 位置模式：SDK 的 kp/kd 是出轴侧口径，转子侧 κ 要乘 r²；
                # 速度模式：官方例程直接把 kd=0.01 当转子侧阻尼用，不再放大
                gain_scale = 1.0 if (args.raw_gains or args.mode == "velocity") else ratio * ratio
                kp_sdk = 0.0 if args.mode == "velocity" else args.kp
                kd_sdk = args.kd if args.kd is not None else \
                    (0.01 if args.mode == "velocity" else 0.05)
                kp_raw = kp_sdk * gain_scale
                kd_raw = kd_sdk * gain_scale
                if kp_raw > GoM8010Can.LIMIT_GAIN or kd_raw > GoM8010Can.LIMIT_GAIN:
                    print(f"换算后 κp={kp_raw:.3f} / κw={kd_raw:.3f} 超过模块上限 "
                          f"{GoM8010Can.LIMIT_GAIN}，请调小 --kp/--kd")
                    return 2
                unit = "rad/s" if args.mode == "velocity" else "rad"
                print(f"当前位置：转子 {(q0 / ratio):.3f} rad（出轴），温度 {feedback.temperature} ℃")
                print(f"模式：{'速度跟随' if args.mode == 'velocity' else '位置跟随'}；"
                      f"轨迹：出轴 ±{amplitude} {unit} @ {args.freq} Hz，"
                      f"渐入 {args.ramp}s，共 {args.duration}s")
                print(f"增益：κp={kp_raw:.3f} κw={kd_raw:.3f}（转子侧）")

                # 2) 换成目标增益
                motor.set_gains(kp_raw, kd_raw)
                time.sleep(0.02)

                # 3) 正弦跟随
                t0 = time.monotonic()
                next_tick = t0
                last_print = 0.0
                omega = 2.0 * math.pi * args.freq
                errors = []  # 渐入结束后的跟踪误差（出轴 rad）
                while True:
                    t = time.monotonic() - t0
                    if t >= args.duration:
                        break
                    if feedback.error is not None:
                        print(f"\n运行中电机报错 {feedback.error}"
                              f"（{_ERROR_TEXT.get(feedback.error, '未知')}），已中止。")
                        break
                    ramp = 1.0 if args.ramp <= 0 else min(1.0, t / args.ramp)
                    torque = "n/a" if feedback.torque is None else f"{feedback.torque:+.3f}"
                    if args.mode == "velocity":
                        # 速度模式：kp 必须为 0，只下发期望速度，位置给 0
                        dq_out = amplitude * ramp * math.sin(omega * t)
                        motor.send(0.0, dq_out * ratio, 0.0)
                        if t >= args.ramp and feedback.velocity is not None:
                            errors.append(dq_out - feedback.velocity / ratio)
                        if t - last_print >= 0.5:
                            last_print = t
                            actual = "n/a" if feedback.velocity is None else \
                                f"{feedback.velocity / ratio:+.3f}"
                            print(f"t={t:5.2f}s  目标={dq_out:+.3f}  实测={actual} rad/s  "
                                  f"力矩={torque} N·m")
                    else:
                        q_out = amplitude * ramp * math.sin(omega * t)
                        dq_out = amplitude * ramp * omega * math.cos(omega * t) \
                            if args.vel_ff else 0.0
                        motor.send(q0 + q_out * ratio, dq_out * ratio, 0.0)
                        if t >= args.ramp and feedback.position is not None:
                            errors.append(q_out - (feedback.position - q0) / ratio)
                        if t - last_print >= 0.5:
                            last_print = t
                            actual = "n/a" if feedback.position is None else \
                                f"{(feedback.position - q0) / ratio:+.3f}"
                            print(f"t={t:5.2f}s  目标={q_out:+.3f}  实测={actual} rad  "
                                  f"力矩={torque} N·m")

                    next_tick += period
                    sleep = next_tick - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_tick = time.monotonic()  # 跟不上就重新对齐，避免堆积

                if errors:
                    rms = math.sqrt(sum(e * e for e in errors) / len(errors))
                    print(f"跟踪误差：RMS={rms:.4f} {unit}，最大={max(abs(e) for e in errors):.4f} {unit} "
                          f"（幅度 {amplitude} {unit} 的 {rms / amplitude * 100:.0f}%）")

                # 4) 收尾：速度归零（位置模式下回中心），再锁定
                for _ in range(20):
                    motor.send(0.0 if args.mode == "velocity" else q0, 0.0, 0.0)
                    time.sleep(period)
                print("轨迹结束，速度已归零。" if args.mode == "velocity" else "轨迹结束，已回到中心位置。")
            finally:
                try:
                    if motor is not None:
                        motor.lock()
                except Exception as exc:  # 锁定时设备可能已断开，不影响退出
                    print(f"锁定失败（忽略）：{exc}")
                subscription.cancel()
                device.disable_channel(args.channel)
        except KeyboardInterrupt:
            print("\n中断，已锁定电机。")
        finally:
            device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
