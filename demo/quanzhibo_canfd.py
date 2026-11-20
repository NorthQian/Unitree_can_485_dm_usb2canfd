"""
@file quanzhibo_canfd.py
@brief 泉智博电机 CAN FD 报文发送示例：标准帧 0x10，仲裁域 1 Mbps / 数据域 5 Mbps，DLC 64。

数据区固定为 `FF FF FF FF FF FF FF FC` 重复 8 次（共 64 字节），即 8 关节使能帧：

    payload = (b"\\xff" * 7 + b"\\xfc") * 8

位定时、终端电阻等公共逻辑在 `qzb_canfd.py`。

用法：
    python3 quanzhibo_canfd.py                      # 连发 10 帧，间隔 100 ms
    python3 quanzhibo_canfd.py --count 1            # 只发一帧
    python3 quanzhibo_canfd.py --count 0            # 一直发，Ctrl-C 停
    python3 quanzhibo_canfd.py --id 0x10 --period 0.05

注意：
    - 本脚本会把通道切换成 CAN FD，模块不支持 FD 时不要用；
    - 改波特率是**运行期**设置，脚本结束时会尝试把通道恢复成原来的配置；
    - CAN FD 帧带 BRS（数据段切到 5M），总线上其它节点也必须支持 CAN FD。
"""

import argparse
import time

from qzb_canfd import configure_canfd, enable_termination, ENABLE_PATTERN

from dmcan import DmCanContext, dmcan_event_flag

CAN_ID = 0x10
"""泉智博电机报文标识符（标准帧）。"""

PAYLOAD = ENABLE_PATTERN * 8
"""64 字节数据区：FF×7 + FC，重复 8 次。"""


def main() -> int:
    """
    @brief 发送泉智博电机报文。

    @return 进程退出码（0 = 发送完成）
    """
    parser = argparse.ArgumentParser(description="泉智博电机 CAN FD 报文示例")
    parser.add_argument("--channel", type=int, default=0, help="CAN 通道号（默认 0）")
    parser.add_argument("--id", type=lambda s: int(s, 0), default=CAN_ID,
                        help="报文标识符（默认 0x10）")
    parser.add_argument("--count", type=int, default=10, help="发送帧数，0 = 一直发（默认 10）")
    parser.add_argument("--period", type=float, default=0.1, help="发送间隔秒（默认 0.1）")
    parser.add_argument("--arb-bitrate", type=int, default=1_000_000,
                        help="仲裁域波特率（默认 1 Mbps）")
    parser.add_argument("--data-bitrate", type=int, default=5_000_000,
                        help="数据域波特率（默认 5 Mbps）")
    parser.add_argument("--arb-sp", type=float, default=0.823,
                        help="仲裁域采样点（默认 0.823，取自说明书）")
    parser.add_argument("--data-sp", type=float, default=0.7647,
                        help="数据域采样点（默认 0.7647，取自说明书）")
    parser.add_argument("--no-restore", action="store_true",
                        help="结束后不恢复通道原来的波特率配置")
    args = parser.parse_args()

    if args.id > 0x7FF:
        print(f"ID 0x{args.id:X} 超出标准帧范围（≤ 0x7FF）")
        return 2

    with DmCanContext() as context:
        if context.find_devices() == 0:
            print("没有找到设备，检查 USB 连接")
            return 2
        device = context.get_device(0)
        device.open()
        original = None
        try:
            # 终端电阻要求通道已使能，所以先开通道再配置波特率
            device.enable_channel(args.channel, True)
            enable_termination(device, args.channel)

            original = device.get_channel_baudrate(args.channel)
            # 通道刚打开或上次异常退出时可能读到全 0，这种配置写回去是非法的
            if original.can_baudrate == 0 or (original.canfd and original.canfd_baudrate == 0):
                original.canfd = False
                original.non_iso = False
                original.retrans = False
                original.can_baudrate = 1_000_000
                original.canfd_baudrate = 0
                original.can_sp = 0.75
                original.canfd_sp = 0.0
                print("读到的原配置无效，结束时将恢复为经典 CAN 1 Mbps")
            device.disable_channel(args.channel)
            configure_canfd(device, args.channel, args.arb_bitrate, args.data_bitrate,
                            args.arb_sp, args.data_sp)
            device.enable_channel(args.channel, True)

            print(f"通道 {args.channel}：CAN FD，仲裁 {args.arb_bitrate // 1000} kbps / "
                  f"数据段 {args.data_bitrate // 1000} kbps，BRS 开启")
            print(f"发送 ID=0x{args.id:X}，DLC=64，标准帧，BRS："
                  f"{PAYLOAD[:8].hex(' ')} ...（共 {len(PAYLOAD)} 字节）")

            bus_errors = []
            error_sub = device.subscribe(dmcan_event_flag.CHANNEL_ERROR,
                                         lambda dev, event: bus_errors.append(
                                             (event.error.lec, event.error.dlec)),
                                         channel_mask=1 << args.channel)
            sent = 0
            try:
                while args.count == 0 or sent < args.count:
                    device.send_can(args.channel, args.id, PAYLOAD,
                                    canfd=True, ext=False, rtr=False, brs=True)
                    sent += 1
                    if args.count != 0:
                        print(f"  第 {sent}/{args.count} 帧已提交")
                    if args.period > 0:
                        time.sleep(args.period)
            except KeyboardInterrupt:
                print("\n已中断")
            time.sleep(0.3)
            error_sub.cancel()
            if bus_errors:
                print(f"共提交 {sent} 帧，但总线上报了 {len(bus_errors)} 次错误：{bus_errors[:5]}")
                print("  lec/dlec 非 0 通常是被对端拒收或没人 ACK，检查波特率 / 采样点 / 接线。")
            else:
                print(f"共提交 {sent} 帧，总线无错误。")
        finally:
            if original is not None and not args.no_restore:
                try:
                    device.disable_channel(args.channel)
                    device.set_channel_baudrate(args.channel, original)
                    time.sleep(0.2)
                    device.enable_channel(args.channel, True)
                    print("已恢复通道原来的波特率配置。")
                except Exception as exc:
                    print(f"恢复原配置失败，回退经典 CAN 1 Mbps：{exc}")
                    try:
                        fallback = device.get_channel_baudrate(args.channel)
                        fallback.canfd = False
                        fallback.can_baudrate = 1_000_000
                        fallback.canfd_baudrate = 0
                        fallback.can_sp = 0.75
                        device.set_channel_baudrate(args.channel, fallback)
                        time.sleep(0.2)
                        device.enable_channel(args.channel, True)
                    except Exception as exc2:
                        print(f"回退也失败：{exc2}")
            device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
