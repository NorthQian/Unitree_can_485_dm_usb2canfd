"""
@file demo.py
@brief DM-DeviceSDK Python 版最小示例：枚举设备 → 打开 → 使能通道 → 订阅收帧 → 发一帧 → 退出。

运行前提：
    1. 装好 Python 包：`pip install ../pack/dmcan_sdk-2.1.1-py3-none-any.whl`（或 `pip install dmcan_sdk`）；
    2. 后端动态库：Windows 下放在本目录的 `dlls/` 里（`libdmcan.dll` 与其依赖 DLL 必须同目录）；
       Linux / macOS 下脚本会按平台与架构自动用本仓库 `C&C++/lib/v2.1.0/<平台>/` 里的库。
       两种情况下都等价于替你设置环境变量 `DMCAN_LIBRARY_PATH`；自行设置该变量可覆盖。

注意：
    - `open()` 会**独占**设备的 UART COM 口（默认自动打开 UART）；
    - 所有接口失败都会抛 `DmCanError`（错误码在 `exc.status`），不再返回 `False`。
"""

import os
import pathlib
import platform
import sys
import time

_DEMO_DIR = pathlib.Path(__file__).resolve().parent
# 仓库根下的预编译库目录：从脚本所在目录往上找，找不到就退回仓库默认布局
_REPO_LIB_DIR = next((p / "C&C++" / "lib" / "v2.1.0"
                      for p in _DEMO_DIR.parents
                      if (p / "C&C++" / "lib" / "v2.1.0").is_dir()),
                     _DEMO_DIR.parents[1] / "C&C++" / "lib" / "v2.1.0")


def _library_candidates() -> list:
    """
    @brief 按平台与架构列出后端动态库的候选路径（按优先级）。

    @return pathlib.Path 列表；Windows 优先用本目录 `dlls/`，Linux 额外找仓库里的预编译库
    """
    if sys.platform == "win32":
        return [_DEMO_DIR / "dlls" / "dmcan.dll", _DEMO_DIR / "dlls" / "libdmcan.dll"]
    if sys.platform == "darwin":
        return [_DEMO_DIR / "dlls" / "libdmcan.dylib",
                _REPO_LIB_DIR / "macos" / "libdmcan.dylib"]
    arch = {"x86_64": "x86_64", "amd64": "x86_64",
            "aarch64": "arm64", "arm64": "arm64"}.get(platform.machine().lower())
    candidates = [_DEMO_DIR / "dlls" / "libdmcan.so"]
    if arch:
        candidates.append(_REPO_LIB_DIR / "linux" / arch / "libdmcan.so")
    return candidates


# 显式指定后端库位置（环境变量优先；都找不到时交给 SDK 自己搜索）
if "DMCAN_LIBRARY_PATH" not in os.environ:
    for _candidate in _library_candidates():
        if _candidate.is_file():
            os.environ["DMCAN_LIBRARY_PATH"] = str(_candidate)
            break

from dmcan import DmCanContext, DmCanError, dmcan_event_flag  # noqa: E402

#------------------------------------------------------------------------------------------- main 
def main() -> int:
    """
    @brief 跑一遍最小示例。

    @return 进程退出码（0 = 成功）
    """
    with DmCanContext() as context:
        print("后端库版本：", context.sdk_version())
        count = context.find_devices()
        print("找到设备：", count, "台")
        context.show_all_devices()
        if count == 0:
            print("没有找到设备，请检查 USB 连接与驱动")
            return 2

        device = context.get_device(0)
        print("设备信息：", device.get_info())
        device.open()
        try:
            print("固件版本：", device.get_version())

            # 通道 0 按设备里已保存的位定时使能（改位定时要先关通道，见 README）
            device.enable_channel(0, True)
            baud = device.get_channel_baudrate(0)
            print("CH0：canfd=%s 标称=%d 数据段=%d" % (bool(baud.canfd), baud.can_baudrate, baud.canfd_baudrate))

            # 收帧推荐走订阅；回调在 SDK 的通知线程上同步执行，必须快速返回
            subscription = device.subscribe(dmcan_event_flag.RX_FRAME,
                                            lambda dev, event: print("  RX", event.frame),
                                            channel_mask=1 << 0)

            # 发一帧：数据长度直接由 payload 决定（2.x 不再单独传 dlen）
            device.send_can(0, 0x123, bytes([0x11, 0x22, 0x33, 0x44]),
                            canfd=bool(baud.canfd), brs=bool(baud.canfd))
            time.sleep(1.0)

            subscription.cancel()
            device.disable_channel(0)
        finally:
            device.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DmCanError as exc:
        print("SDK 调用失败：%s（错误码 %s）" % (exc, exc.status))
        raise SystemExit(1)
