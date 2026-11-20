"""
@file qzb_gui.py
@brief 泉智博电机位置控制上位机：三个滑块分别控制 ID 1/2/3 的位置（CAN FD MIT 模式）。

界面：
    - 每台电机一个滑块，范围 -12.5 ~ +12.5 rad；
    - 可调 Kp / Kd / 目标位置的变化速率（限速，避免拖动滑块时电机猛冲）；
    - 实时显示各电机的实际位置、速度、温度、使能状态和错误码。

运行：
    python3 qzb_gui.py
    python3 qzb_gui.py --ids 1,2,3 --slots 0,1,2

离开界面时会先发失能帧再关闭通道。若电机已经因 CAN 超时锁存（err 0x2000），
需要先给电机断电重上电。

依赖：PyQt5（系统包 python3-pyqt5，venv 需开启 include-system-site-packages）。
"""

import argparse
import math
import os
import sys
import threading
import time


def _sanitize_snap_env() -> None:
    """
    @brief 清掉 VS Code（snap 版）集成终端注入的 GTK / GIO 路径。

    @note Qt 创建 QApplication 时会自动加载 GTK3 平台主题插件，顺着 `GTK_PATH`、
          `GIO_MODULE_DIR` 等变量去加载 snap 内的 GTK 库，与系统 glibc 冲突，报
          `__libc_pthread_init / GLIBC_PRIVATE` 符号错误。这些变量只对 snap 内的程序有意义，
          对本进程去掉即可，Qt 会回退到系统 GTK/字体配置。
    """
    for name in ("GTK_PATH", "GTK_EXE_PREFIX", "GIO_MODULE_DIR", "GTK_IM_MODULE_FILE",
                 "GSETTINGS_SCHEMA_DIR", "LOCPATH"):
        if "/snap" in os.environ.get(name, ""):
            os.environ.pop(name, None)
    data_dirs = os.environ.get("XDG_DATA_DIRS", "")
    if "/snap" in data_dirs:
        kept = [item for item in data_dirs.split(":") if "/snap" not in item]
        os.environ["XDG_DATA_DIRS"] = ":".join(kept) or "/usr/local/share:/usr/share"


_sanitize_snap_env()

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                             QPushButton, QSlider, QVBoxLayout, QWidget, QDoubleSpinBox)

from qzb_canfd import (DISABLE_PATTERN, ENABLE_PATTERN, ERROR_TEXT,
                       configure_canfd, enable_termination, pack_joint, unpack_feedback)

from dmcan import DmCanContext, dmcan_event_flag

SLIDER_SCALE = 100  # 滑块 1 格 = 0.01 rad


class SharedState:
    """@brief GUI 线程与 CAN 线程共享的状态（锁保护）。"""

    def __init__(self, motor_ids, slots, channel):
        self.lock = threading.Lock()
        self.motor_ids = motor_ids
        self.slot_of = dict(zip(motor_ids, slots))
        self.channel = channel
        self.targets = {mid: 0.0 for mid in motor_ids}
        self.command = {mid: 0.0 for mid in motor_ids}
        self.initialized = {mid: False for mid in motor_ids}
        self.latest = {mid: None for mid in motor_ids}
        self.slider_dirty = {mid: False for mid in motor_ids}
        self.kp = 20.0
        self.kd = 2.0
        self.slew = 3.0  # rad/s，目标位置的变化上限
        self.running = False
        self.status = "未连接"


class MotorWorker(threading.Thread):
    """@brief 后台 CAN 线程：配置通道、使能、按限速更新位置指令并周期下发。"""

    def __init__(self, shared: SharedState):
        super().__init__(daemon=True)
        self.shared = shared
        self.stop_event = threading.Event()

    def stop(self) -> None:
        """@brief 请求线程退出。"""
        self.stop_event.set()

    def run(self) -> None:
        """@brief 线程主体。"""
        sh = self.shared
        try:
            with DmCanContext() as context:
                if context.find_devices() == 0:
                    sh.status = "未找到 USB-CAN 设备"
                    return
                device = context.get_device(0)
                device.open()
                original = None
                try:
                    device.enable_channel(sh.channel, True)
                    enable_termination(device, sh.channel)
                    original = device.get_channel_baudrate(sh.channel)
                    if original.can_baudrate == 0 or \
                            (original.canfd and original.canfd_baudrate == 0):
                        original.canfd = False
                        original.can_baudrate = 1_000_000
                        original.canfd_baudrate = 0
                        original.can_sp = 0.75
                    device.disable_channel(sh.channel)
                    configure_canfd(device, sh.channel, 1_000_000, 5_000_000, 0.823, 0.7647)
                    device.enable_channel(sh.channel, True)

                    def on_frame(dev, event):
                        frame = event.frame
                        if frame.can_id in sh.latest:  # 值可能还是 None，不能用 get() 判断
                            data = bytes(frame.payload[: frame.dlen])
                            if len(data) >= 8:
                                sh.latest[frame.can_id] = unpack_feedback(data)

                    subscription = device.subscribe(dmcan_event_flag.RX_FRAME, on_frame,
                                                    channel_mask=1 << sh.channel)
                    try:
                        for _ in range(10):
                            device.send_can(sh.channel, 0x10, ENABLE_PATTERN * 8,
                                            canfd=True, brs=True)
                            time.sleep(0.02)
                        sh.status = "已连接，等待反馈"

                        period = 0.005  # 200 Hz
                        last = time.monotonic()
                        while not self.stop_event.is_set():
                            now = time.monotonic()
                            dt = min(now - last, 0.05)
                            last = now

                            with sh.lock:
                                targets = dict(sh.targets)
                                kp, kd, slew = sh.kp, sh.kd, sh.slew
                                commands = dict(sh.command)
                                ready = dict(sh.initialized)
                                dirty = dict(sh.slider_dirty)
                                latest = {mid: sh.latest[mid] for mid in sh.motor_ids}

                            # 首次收到反馈时，把指令初始化到当前位置，避免上电猛冲
                            for mid in sh.motor_ids:
                                if not ready[mid] and latest[mid] is not None:
                                    commands[mid] = latest[mid]["pos"]
                                    ready[mid] = True
                                    targets[mid] = commands[mid]
                            if all(ready.values()) and sh.status.startswith("已连接"):
                                sh.status = "控制中"

                            # 按限速把指令推向目标
                            for mid in sh.motor_ids:
                                if not ready[mid]:
                                    continue
                                step = slew * dt
                                delta = targets[mid] - commands[mid]
                                if abs(delta) <= step:
                                    commands[mid] = targets[mid]
                                else:
                                    commands[mid] += math.copysign(step, delta)

                            with sh.lock:
                                sh.command = commands
                                sh.initialized = ready
                                # 用户没拖过滑条的电机，目标跟随当前位置（滑条同步回写）
                                for mid in sh.motor_ids:
                                    if not dirty[mid]:
                                        sh.targets[mid] = commands[mid]

                            # 没拿到反馈的电机先填零增益帧：不产生力矩，但会让它回反馈
                            frame = bytearray(64)
                            for mid in sh.motor_ids:
                                slot = sh.slot_of[mid]
                                if ready[mid]:
                                    frame[slot * 8:(slot + 1) * 8] = pack_joint(
                                        commands[mid], 0.0, kp, kd, 0.0)
                                else:
                                    frame[slot * 8:(slot + 1) * 8] = pack_joint(
                                        0.0, 0.0, 0.0, 0.0, 0.0)
                            device.send_can(sh.channel, 0x20, bytes(frame),
                                            canfd=True, brs=True)

                            time.sleep(period)
                            if self.stop_event.is_set():
                                break
                        sh.status = "已停止"
                    finally:
                        subscription.cancel()
                        for _ in range(5):
                            device.send_can(sh.channel, 0x10, DISABLE_PATTERN * 8,
                                            canfd=True, brs=True)
                            time.sleep(0.02)
                finally:
                    if original is not None:
                        try:
                            device.disable_channel(sh.channel)
                            device.set_channel_baudrate(sh.channel, original)
                            time.sleep(0.2)
                            device.enable_channel(sh.channel, True)
                        except Exception:
                            pass
                    device.close()
        except Exception as exc:
            sh.status = f"错误：{exc}"
        finally:
            sh.running = False


class MotorRow(QGroupBox):
    """@brief 单台电机的控制行：滑块 + 目标值 + 实际反馈。"""

    def __init__(self, motor_id: int, state: SharedState):
        super().__init__(f"电机 ID {motor_id}")
        self.motor_id = motor_id
        self.state = state

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-1250, 1250)  # ±12.5 rad，步进 0.01
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(250)
        self.slider.valueChanged.connect(self._on_slider)

        # 直接输入目标位置（回车或失焦才生效，避免边打字边下发）
        self.input = QDoubleSpinBox()
        self.input.setRange(-12.5, 12.5)
        self.input.setDecimals(2)
        self.input.setSingleStep(0.1)
        self.input.setSuffix(" rad")
        self.input.setKeyboardTracking(False)
        self.input.setMinimumWidth(120)
        self.input.valueChanged.connect(self._on_input)

        self.feedback_label = QLabel("—")
        self.feedback_label.setMinimumWidth(260)

        layout = QHBoxLayout(self)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.input)
        layout.addWidget(self.feedback_label)

    def _set_target(self, target: float) -> None:
        """@brief 写入目标位置并同步滑块 / 输入框。"""
        with self.state.lock:
            self.state.targets[self.motor_id] = target
            self.state.slider_dirty[self.motor_id] = True
        for widget, value in ((self.slider, int(round(target * SLIDER_SCALE))),
                              (self.input, target)):
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)

    def _on_slider(self, value: int) -> None:
        """@brief 拖动滑块：更新目标。"""
        self._set_target(value / SLIDER_SCALE)

    def _on_input(self, value: float) -> None:
        """@brief 输入框回车 / 失焦：更新目标。"""
        self._set_target(value)

    def refresh(self) -> None:
        """@brief 定时刷新目标与实际反馈。"""
        with self.state.lock:
            target = self.state.targets[self.motor_id]
            dirty = self.state.slider_dirty[self.motor_id]
            data = self.state.latest[self.motor_id]
        if not dirty:
            # 还没被用户改过时，滑块和输入框跟随电机实际位置
            if abs(self.slider.value() / SLIDER_SCALE - target) > 0.005:
                self.slider.blockSignals(True)
                self.slider.setValue(int(round(target * SLIDER_SCALE)))
                self.slider.blockSignals(False)
            if abs(self.input.value() - target) > 0.005:
                self.input.blockSignals(True)
                self.input.setValue(target)
                self.input.blockSignals(False)

        if data is None:
            self.feedback_label.setText("实际 —")
            return
        err = data["err"]
        faults = [text for bit, text in ERROR_TEXT.items()
                  if err & bit and bit != (1 << 0)]
        status = "使能" if data["enabled"] else "未使能"
        if faults:
            status += " / " + "、".join(faults)
        self.feedback_label.setText(
            f"实际 {data['pos']:+.2f} rad  {data['vel']:+.1f} rad/s  "
            f"{data['temp']:.0f}℃  {status}")


class MainWindow(QMainWindow):
    """@brief 主窗口。"""

    def __init__(self, state: SharedState):
        super().__init__()
        self.state = state
        self.worker = None

        self.setWindowTitle("泉智博电机三轴位置控制")
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # 参数栏
        params = QHBoxLayout()
        self.kp_spin = self._add_spin(params, "Kp", 0.0, 500.0, state.kp, 1.0,
                                      lambda v: setattr(state, "kp", v))
        self.kd_spin = self._add_spin(params, "Kd", 0.0, 5.0, state.kd, 0.1,
                                      lambda v: setattr(state, "kd", v))
        self.slew_spin = self._add_spin(params, "限速 rad/s", 0.1, 65.0, state.slew, 0.5,
                                        lambda v: setattr(state, "slew", v))
        self.start_button = QPushButton("开始控制")
        self.start_button.clicked.connect(self._toggle)
        params.addWidget(self.start_button)
        root.addLayout(params)

        # 三台电机的控制行
        self.rows = []
        for mid in state.motor_ids:
            row = MotorRow(mid, state)
            self.rows.append(row)
            root.addWidget(row)

        self.status_label = QLabel("状态：未连接")
        root.addWidget(self.status_label)
        self.resize(980, 320)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)

    def _add_spin(self, layout, name, lo, hi, value, step, on_change):
        """
        @brief 往布局里加一个「标签 + 数值框」。

        @param layout 目标布局
        @param name 标签文字
        @param lo 最小值
        @param hi 最大值
        @param value 初始值
        @param step 步进
        @param on_change 值变化回调
        @return 数值框
        """
        layout.addWidget(QLabel(name))
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(2)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setMinimumWidth(90)
        spin.valueChanged.connect(on_change)
        layout.addWidget(spin)
        return spin

    def _toggle(self) -> None:
        """@brief 开始 / 停止控制。"""
        if self.worker is not None and self.state.running:
            self.worker.stop()
            self.worker.join(timeout=3.0)
            self.worker = None
            self.start_button.setText("开始控制")
            return
        self.worker = MotorWorker(self.state)
        self.state.running = True
        self.worker.start()
        self.start_button.setText("停止控制")

    def _refresh(self) -> None:
        """@brief 定时刷新界面。"""
        for row in self.rows:
            row.refresh()
        self.status_label.setText(f"状态：{self.state.status}")
        if not self.state.running and self.worker is not None:
            self.worker = None
            self.start_button.setText("开始控制")

    def closeEvent(self, event) -> None:
        """@brief 关窗前停止线程，确保失能帧发出。"""
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(timeout=3.0)
        event.accept()


def main() -> int:
    """
    @brief 入口。

    @return 进程退出码
    """
    parser = argparse.ArgumentParser(description="泉智博电机三轴位置控制（PyQt5）")
    parser.add_argument("--ids", type=lambda s: [int(x, 0) for x in s.split(",") if x.strip()],
                        default=[1, 2, 3], help="电机反馈 ID 列表（默认 1,2,3）")
    parser.add_argument("--slots", type=lambda s: [int(x, 0) for x in s.split(",") if x.strip()],
                        default=None, help="MIT 槽位列表（默认 ID-1）")
    parser.add_argument("--channel", type=int, default=0, help="CAN 通道号（默认 0）")
    args = parser.parse_args()

    slots = args.slots if args.slots is not None else [mid - 1 for mid in args.ids]
    if len(slots) != len(args.ids):
        print("--slots 的个数必须与 --ids 一致")
        return 2

    app = QApplication(sys.argv)
    state = SharedState(args.ids, slots, args.channel)
    window = MainWindow(state)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
