#!/usr/bin/env python3
import argparse
import math
import os
import signal
import sys
import time
from threading import Lock

from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool, Float32, Int32


def _import_qt():
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets

        return QtCore, QtGui, QtWidgets, "PyQt5"
    except ImportError:
        pass

    try:
        from PySide6 import QtCore, QtGui, QtWidgets

        return QtCore, QtGui, QtWidgets, "PySide6"
    except ImportError:
        pass

    try:
        from PySide2 import QtCore, QtGui, QtWidgets

        return QtCore, QtGui, QtWidgets, "PySide2"
    except ImportError as exc:
        raise RuntimeError(
            "Qt Python bindings were not found. Install PyQt5, PySide6, or PySide2."
        ) from exc


QtCore, QtGui, QtWidgets, QT_BINDING = _import_qt()


def qt_key_value(key):
    try:
        return int(key)
    except TypeError:
        return key.value


def qt_enum(name, group_name=None):
    if hasattr(QtCore.Qt, name):
        return getattr(QtCore.Qt, name)
    if group_name and hasattr(QtCore.Qt, group_name):
        group = getattr(QtCore.Qt, group_name)
        if hasattr(group, name):
            return getattr(group, name)
    raise AttributeError(f"Qt enum '{name}' was not found")


def qt_exec(app):
    if hasattr(app, "exec"):
        return app.exec()
    return app.exec_()


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


def move_towards(current, target, max_delta):
    if current < target:
        return min(current + max_delta, target)
    if current > target:
        return max(current - max_delta, target)
    return target


def topic_join(namespace, suffix):
    return f"{namespace.rstrip('/')}/{suffix.lstrip('/')}"


def format_seconds(value):
    if value is None or not math.isfinite(value):
        return "--:--.---"
    minutes = int(value // 60)
    seconds = value - minutes * 60
    return f"{minutes:02d}:{seconds:06.3f}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Qt Widgets AutoDRIVE and F1TENTH teleop dashboard.",
    )
    parser.add_argument(
        "--ros",
        choices=("auto", "1", "2"),
        default="2",
        help="ROS version to target (default: 2).",
    )
    parser.add_argument(
        "--mode",
        choices=("autonomous", "manual"),
        default="autonomous",
        help="Initial command output mode (default: %(default)s).",
    )
    parser.add_argument(
        "--f1tenth",
        action="store_true",
        help=(
            "Publish steer/throttle as Joy messages instead of AutoDRIVE command topics. "
            "ROS1 publishes /vesc/joy; ROS2 publishes /joy."
        ),
    )
    parser.add_argument(
        "--namespace",
        default="/autodrive/roboracer_1",
        help="AutoDRIVE vehicle namespace (default: %(default)s).",
    )
    parser.add_argument("--camera-topic", default=None, help="Front camera Image topic.")
    parser.add_argument("--speed-topic", default=None, help="Speed Float32 topic.")
    parser.add_argument("--lap-time-topic", default=None, help="Lap time Float32 topic.")
    parser.add_argument("--last-lap-topic", default=None, help="Last lap Float32 topic.")
    parser.add_argument("--best-lap-topic", default=None, help="Best lap Float32 topic.")
    parser.add_argument("--collision-count-topic", default=None, help="Collision count Int32 topic.")
    parser.add_argument("--rate", type=float, default=20.0, help="Command publish rate in Hz.")
    parser.add_argument(
        "--autodrive-throttle-scale",
        type=float,
        default=0.15,
        help="Scale applied to native AutoDRIVE throttle commands (default: %(default)s).",
    )
    parser.add_argument(
        "--autodrive-steering-scale",
        type=float,
        default=0.8,
        help="Scale applied to native AutoDRIVE steering commands (default: %(default)s).",
    )
    parser.add_argument(
        "--throttle-step",
        type=float,
        default=0.80,
        help="Throttle target ramp in command-units per second.",
    )
    parser.add_argument(
        "--steer-step",
        type=float,
        default=1.40,
        help="Steering target ramp in command-units per second.",
    )
    return parser.parse_args()


def resolve_ros_version(requested):
    if requested in ("1", "2"):
        return int(requested)

    env_version = os.environ.get("ROS_VERSION")
    if env_version in ("1", "2"):
        return int(env_version)

    try:
        import rclpy  # noqa: F401

        return 2
    except ImportError:
        pass

    try:
        import rospy  # noqa: F401

        return 1
    except ImportError:
        pass

    raise RuntimeError("Unable to detect ROS version. Export ROS_VERSION or pass --ros {1,2}.")


class TeleopState:
    KEY_UP = "up"
    KEY_DOWN = "down"
    KEY_LEFT = "left"
    KEY_RIGHT = "right"

    def __init__(self, args):
        self.args = args
        self.pressed_keys = set()
        self.throttle = 0.0
        self.steering = 0.0
        self.deadman_released = True
        self.reset_pending = False
        self.last_update_ts = time.monotonic()

    def set_key(self, key_name, pressed):
        if pressed:
            self.pressed_keys.add(key_name)
        else:
            self.pressed_keys.discard(key_name)

    def stop(self):
        self.pressed_keys.clear()
        self.throttle = 0.0
        self.steering = 0.0

    def update(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_update_ts)
        self.last_update_ts = now

        up = self.KEY_UP in self.pressed_keys
        down = self.KEY_DOWN in self.pressed_keys
        left = self.KEY_LEFT in self.pressed_keys
        right = self.KEY_RIGHT in self.pressed_keys

        target_throttle = 0.0
        if up != down:
            target_throttle = 1.0 if up else -1.0

        target_steering = 0.0
        if left != right:
            target_steering = 1.0 if left else -1.0

        self.throttle = clamp(
            move_towards(self.throttle, target_throttle, self.args.throttle_step * dt)
        )
        self.steering = clamp(
            move_towards(self.steering, target_steering, self.args.steer_step * dt)
        )


class SteeringWheel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.steering = 0.0
        self.setFixedSize(106, 106)
        self.setAttribute(qt_enum("WA_TranslucentBackground", "WidgetAttribute"), True)
        self.setStyleSheet("background: transparent;")

    def set_steering(self, steering):
        self.steering = float(steering)
        self.update()

    def paintEvent(self, event):
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self.steering * -45.0)

        radius = min(self.width(), self.height()) * 0.43
        painter.setPen(QtGui.QPen(QtGui.QColor("#111417"), 9, qt_enum("SolidLine", "PenStyle"), qt_enum("RoundCap", "PenCapStyle")))
        painter.drawEllipse(QtCore.QPointF(0, 0), radius, radius)
        painter.setPen(QtGui.QPen(QtGui.QColor("#c5cbd1"), 4))
        painter.drawEllipse(QtCore.QPointF(0, 0), radius, radius)

        painter.setPen(QtGui.QPen(QtGui.QColor("#171b1f"), 7, qt_enum("SolidLine", "PenStyle"), qt_enum("RoundCap", "PenCapStyle")))
        for x, y in ((0, -radius + 10), (-radius + 12, radius * 0.35), (radius - 12, radius * 0.35)):
            painter.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(x, y))

        painter.setPen(QtGui.QPen(QtGui.QColor("#d8dee4"), 2))
        painter.setBrush(QtGui.QBrush(QtGui.QColor("#242a2f")))
        painter.drawEllipse(QtCore.QPointF(0, 0), 13, 13)


class ThrottleGauge(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.throttle = 0.0
        self.setFixedSize(16, 50)
        self.setAttribute(qt_enum("WA_TranslucentBackground", "WidgetAttribute"), True)
        self.setStyleSheet("background: transparent;")

    def set_throttle(self, throttle):
        self.throttle = float(throttle)
        self.update()

    def paintEvent(self, event):
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        rect = self.rect().adjusted(0, 0, -1, -1)
        painter.setPen(QtGui.QPen(QtGui.QColor("#777f86"), 1))
        painter.setBrush(QtGui.QBrush(QtGui.QColor("#30363b")))
        painter.drawRoundedRect(rect, 2, 2)

        mid = self.height() / 2
        painter.setPen(QtGui.QPen(QtGui.QColor("#9da7af"), 1))
        painter.drawLine(1, int(mid), self.width() - 2, int(mid))

        fill_max = mid - 4
        fill = max(2.0, abs(self.throttle) * fill_max) if abs(self.throttle) > 0.01 else 0.0
        if fill <= 0:
            return
        y = mid - fill if self.throttle >= 0 else mid
        painter.setPen(qt_enum("NoPen", "PenStyle"))
        painter.setBrush(QtGui.QBrush(QtGui.QColor("#8cff6a" if self.throttle >= 0 else "#ff5c5c")))
        painter.drawRoundedRect(QtCore.QRectF(3, y, self.width() - 6, fill), 2, 2)


class AutoDriveTeleopWindow(QtWidgets.QWidget):
    def __init__(self, args, ros_version):
        super().__init__()
        self.args = args
        self.ros_version = ros_version
        settings_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            ".autodrive_teleop.ini",
        )
        self.settings = QtCore.QSettings(settings_path, QtCore.QSettings.IniFormat)
        self.state = TeleopState(args)
        self.mode = args.mode
        self.speed = 0.0
        self.lap_time = None
        self.last_lap = None
        self.best_lap = None
        self.latest_camera = None
        self.front_camera_enabled = False
        self.ros = None
        self.publish_period = 1.0 / max(args.rate, 1.0)
        self.next_publish_ts = time.monotonic()
        self._closed = False
        self._dragging = False
        self._drag_offset = QtCore.QPoint(0, 0)
        self._restoring_settings = False
        self._settings_ready = False

        self._build_ui()
        self._init_key_map()
        self._settings_ready = True

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(10)

    def _build_ui(self):
        self.setWindowTitle("AutoDRIVE TeleOp")
        self.setFixedSize(400, 240)
        self.setFocusPolicy(qt_enum("StrongFocus", "FocusPolicy"))
        self.setStyleSheet(
            """
            QWidget { background: #0b0d0f; color: #f4f7fa; font-family: Arial; }
            QLabel#camera { background: #15191d; color: #64707a; font-size: 18px; font-weight: 700; }
            QLabel#dash { background: rgba(77, 82, 88, 204); }
            QLabel#glass { background: rgba(0, 0, 0, 51); border-radius: 4px; }
            QLabel#manual { background: transparent; color: #273039; font-size: 13px; font-weight: 700; }
            QLabel#speed { background: transparent; color: #f4f7fa; font-family: 'DejaVu Sans Mono', monospace; font-size: 28px; font-weight: 700; }
            QLabel#lapName { background: transparent; color: #c7d0d8; font-size: 10px; }
            QLabel#lapValue { background: transparent; color: #f4f7fa; font-family: 'DejaVu Sans Mono', monospace; font-size: 10px; font-weight: 700; }
            QLabel#gearOnD { color: #8cff6a; font-size: 24px; font-weight: 700; background: transparent; }
            QLabel#gearOnR { color: #ff5c5c; font-size: 24px; font-weight: 700; background: transparent; }
            QLabel#gearOff { color: #2a3035; font-size: 24px; font-weight: 700; background: transparent; }
            QCheckBox { background: transparent; color: #f4f7fa; font-size: 10px; font-weight: 700; spacing: 5px; }
            QCheckBox::indicator {
                width: 11px;
                height: 11px;
                border: 1px solid #f4f7fa;
                background: rgba(0, 0, 0, 80);
            }
            QCheckBox::indicator:checked {
                background: #8cff6a;
                border: 1px solid #132012;
            }
            QPushButton { background: #3a4046; color: #cfd7de; border: 1px solid #838b92; border-radius: 9px; padding: 3px 6px; font-size: 10px; font-weight: 700; }
            QPushButton:checked { background: #8cff6a; color: #152012; border-color: #b8ffab; }
            """
        )

        self.camera_label = QtWidgets.QLabel("NO CAMERA", self)
        self.camera_label.setObjectName("camera")
        self.camera_label.setGeometry(0, 0, 400, 240)
        self.camera_label.setAlignment(qt_enum("AlignCenter", "AlignmentFlag"))
        self.camera_label.setScaledContents(False)

        self.manual_label = QtWidgets.QLabel(
            "Keyboard Shortcuts\n"
            "Left / Right : Steering wheel\n"
            "Up / Down : Drive / Reverse throttle\n"
            "A : Toggle to Autonomous Drive",
            self,
        )
        self.manual_label.setObjectName("manual")
        self.manual_label.setGeometry(0, 52, 400, 92)
        self.manual_label.setAlignment(qt_enum("AlignCenter", "AlignmentFlag"))
        self.manual_label.hide()

        self.dashboard_bg = QtWidgets.QLabel(self)
        self.dashboard_bg.setObjectName("dash")
        self.dashboard_bg.setGeometry(0, 168, 400, 72)

        self.speed_panel = QtWidgets.QLabel(self)
        self.speed_panel.setObjectName("glass")
        self.speed_panel.setGeometry(8, 7, 100, 36)
        self.speed_label = QtWidgets.QLabel("000.0", self)
        self.speed_label.setObjectName("speed")
        self.speed_label.setGeometry(14, 8, 88, 34)
        self.speed_label.setAlignment(qt_enum("AlignCenter", "AlignmentFlag"))

        self.lap_panel = QtWidgets.QLabel(self)
        self.lap_panel.setObjectName("glass")
        self.lap_panel.setGeometry(252, 7, 140, 45)
        self.lap_name_label = QtWidgets.QLabel("Lap time", self)
        self.last_lap_name_label = QtWidgets.QLabel("Last Lap", self)
        self.best_lap_name_label = QtWidgets.QLabel("Best Lap", self)
        self.lap_value_label = QtWidgets.QLabel(self)
        self.last_lap_value_label = QtWidgets.QLabel(self)
        self.best_lap_value_label = QtWidgets.QLabel(self)
        for y, name_label, value_label in (
            (10, self.lap_name_label, self.lap_value_label),
            (24, self.last_lap_name_label, self.last_lap_value_label),
            (38, self.best_lap_name_label, self.best_lap_value_label),
        ):
            name_label.setObjectName("lapName")
            name_label.setGeometry(260, y, 48, 12)
            value_label.setObjectName("lapValue")
            value_label.setGeometry(306, y, 78, 12)
            value_label.setAlignment(qt_enum("AlignRight", "AlignmentFlag"))

        self.wheel = SteeringWheel(self)
        self.wheel.move(42, 136)

        self.gauge = ThrottleGauge(self)
        self.gauge.move(154, 177)
        self.drive_label = QtWidgets.QLabel("D", self)
        self.reverse_label = QtWidgets.QLabel("R", self)
        self.drive_label.setGeometry(176, 176, 24, 24)
        self.reverse_label.setGeometry(176, 203, 24, 24)

        self.front_camera_checkbox = QtWidgets.QCheckBox("Front Camera", self)
        self.front_camera_checkbox.setGeometry(252, 174, 140, 16)
        self.front_camera_checkbox.setChecked(False)
        self.front_camera_checkbox.toggled.connect(self._set_front_camera_enabled)

        self.deadman_checkbox = QtWidgets.QCheckBox("Deadman Release", self)
        self.deadman_checkbox.setGeometry(252, 193, 140, 16)
        self.deadman_checkbox.setChecked(True)
        self.deadman_checkbox.toggled.connect(self._set_deadman_released)

        self.mode_button = QtWidgets.QPushButton(self)
        self.mode_button.setCheckable(True)
        self.mode_button.setGeometry(252, 212, 140, 20)
        self.mode_button.clicked.connect(self._toggle_mode_from_button)

        self._restore_settings()
        self._install_drag_filters()
        self._refresh_ui()

    def _init_key_map(self):
        self.key_map = {
            qt_key_value(qt_enum("Key_Up", "Key")): TeleopState.KEY_UP,
            qt_key_value(qt_enum("Key_Down", "Key")): TeleopState.KEY_DOWN,
            qt_key_value(qt_enum("Key_Left", "Key")): TeleopState.KEY_LEFT,
            qt_key_value(qt_enum("Key_Right", "Key")): TeleopState.KEY_RIGHT,
        }

    def attach_ros(self, ros):
        self.ros = ros

    def set_camera_image(self, image):
        self.latest_camera = image

    def set_speed(self, value):
        self.speed = float(value)

    def set_lap_time(self, value):
        self.lap_time = float(value)

    def set_last_lap(self, value):
        self.last_lap = float(value)

    def set_best_lap(self, value):
        self.best_lap = float(value)

    def set_collision_count(self, value):
        del value
        if self.state.deadman_released and self.mode == "autonomous":
            self.set_mode("manual")

    def _toggle_mode_from_button(self, checked=False):
        self.set_mode("autonomous" if checked else "manual")

    def _set_front_camera_enabled(self, enabled):
        self.front_camera_enabled = bool(enabled)
        self._refresh_ui()
        self._save_settings()
        self.setFocus(qt_enum("OtherFocusReason", "FocusReason"))

    def _set_deadman_released(self, enabled):
        self.state.deadman_released = bool(enabled)
        self._refresh_ui()
        self._save_settings()
        self.setFocus(qt_enum("OtherFocusReason", "FocusReason"))

    def _settings_bool(self, name, default):
        value = self.settings.value(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _restore_settings(self):
        self._restoring_settings = True
        geometry = self.settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)

        front_camera = self._settings_bool("front_camera_enabled", False)
        deadman_released = self._settings_bool("deadman_released", True)
        self.front_camera_checkbox.blockSignals(True)
        self.front_camera_checkbox.setChecked(front_camera)
        self.front_camera_checkbox.blockSignals(False)
        self.front_camera_enabled = front_camera

        self.deadman_checkbox.blockSignals(True)
        self.deadman_checkbox.setChecked(deadman_released)
        self.deadman_checkbox.blockSignals(False)
        self.state.deadman_released = deadman_released

        if not getattr(self.args, "mode_explicit", False):
            saved_mode = self.settings.value("drive_mode", self.mode)
            saved_mode = {
                "autodrive": "autonomous",
                "humandrive": "manual",
                "f1tenth": "manual",
            }.get(saved_mode, saved_mode)
            if saved_mode in ("autonomous", "manual"):
                self.mode = saved_mode
        self._restoring_settings = False

    def _save_settings(self):
        if self._restoring_settings or not self._settings_ready:
            return
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("front_camera_enabled", self.front_camera_checkbox.isChecked())
        self.settings.setValue("deadman_released", self.deadman_checkbox.isChecked())
        self.settings.setValue("drive_mode", self.mode)
        self.settings.sync()

    def set_mode(self, mode):
        if mode not in ("autonomous", "manual") or mode == self.mode:
            return
        previous_mode = self.mode
        self.state.stop()
        if previous_mode == "manual" and self.ros is not None:
            self.ros.publish_neutral(previous_mode)
        self.mode = mode
        self._refresh_ui()
        self._save_settings()

    def _refresh_ui(self):
        if not self.front_camera_enabled:
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText("")
            self.camera_label.setStyleSheet("background: #ffffff;")
            self.manual_label.show()
        elif self.latest_camera is not None:
            self.camera_label.setStyleSheet("")
            self.manual_label.hide()
            pixmap = QtGui.QPixmap.fromImage(self.latest_camera)
            pixmap = pixmap.scaled(
                self.camera_label.size(),
                qt_enum("KeepAspectRatioByExpanding", "AspectRatioMode"),
                qt_enum("SmoothTransformation", "TransformationMode"),
            )
            x = max(0, (pixmap.width() - self.camera_label.width()) // 2)
            y = max(0, (pixmap.height() - self.camera_label.height()) // 2)
            pixmap = pixmap.copy(x, y, self.camera_label.width(), self.camera_label.height())
            self.camera_label.setPixmap(pixmap)
        else:
            self.camera_label.setStyleSheet("")
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText("NO CAMERA")
            self.manual_label.hide()

        self.speed_label.setText(f"{self.speed:05.1f}")
        self.lap_value_label.setText(format_seconds(self.lap_time))
        self.last_lap_value_label.setText(format_seconds(self.last_lap))
        self.best_lap_value_label.setText(format_seconds(self.best_lap))
        lap_name_color = "#c7d0d8" if self.front_camera_enabled else "#273039"
        lap_value_color = "#f4f7fa" if self.front_camera_enabled else "#111820"
        self.speed_label.setStyleSheet(
            "background: transparent; "
            f"color: {lap_value_color}; "
            "font-family: 'DejaVu Sans Mono', monospace; font-size: 28px; font-weight: 700;"
        )
        for label in (self.lap_name_label, self.last_lap_name_label, self.best_lap_name_label):
            label.setStyleSheet(
                f"background: transparent; color: {lap_name_color}; font-size: 10px;"
            )
        for label in (self.lap_value_label, self.last_lap_value_label, self.best_lap_value_label):
            label.setStyleSheet(
                "background: transparent; "
                f"color: {lap_value_color}; "
                "font-family: 'DejaVu Sans Mono', monospace; font-size: 10px; font-weight: 700;"
            )
        self.wheel.set_steering(self.state.steering)
        self.gauge.set_throttle(self.state.throttle)

        self.drive_label.setObjectName("gearOnD" if self.state.throttle >= -0.02 else "gearOff")
        self.reverse_label.setObjectName("gearOnR" if self.state.throttle < -0.02 else "gearOff")
        for label in (self.drive_label, self.reverse_label):
            label.style().unpolish(label)
            label.style().polish(label)

        self.mode_button.blockSignals(True)
        self.mode_button.setChecked(self.mode == "autonomous")
        self.mode_button.setText("Autonomous Drive" if self.mode == "autonomous" else "Manual Drive")
        self.mode_button.blockSignals(False)

    def _install_drag_filters(self):
        for widget in (
            self,
            self.camera_label,
            self.manual_label,
            self.dashboard_bg,
            self.speed_panel,
            self.speed_label,
            self.lap_panel,
            self.lap_name_label,
            self.last_lap_name_label,
            self.best_lap_name_label,
            self.lap_value_label,
            self.last_lap_value_label,
            self.best_lap_value_label,
            self.wheel,
            self.drive_label,
            self.reverse_label,
            self.gauge,
        ):
            widget.installEventFilter(self)

    def _global_pos(self, event):
        if hasattr(event, "globalPosition"):
            return event.globalPosition().toPoint()
        return event.globalPos()

    def eventFilter(self, obj, event):
        event_type = event.type()
        if event_type == qevent_type("MouseButtonPress") and event.button() == qt_enum("LeftButton", "MouseButton"):
            self._dragging = True
            self._drag_offset = self._global_pos(event) - self.frameGeometry().topLeft()
            self.setFocus(qt_enum("MouseFocusReason", "FocusReason"))
            event.accept()
            return True
        if event_type == qevent_type("MouseMove") and self._dragging:
            buttons = event.buttons()
            if buttons & qt_enum("LeftButton", "MouseButton"):
                self.move(self._global_pos(event) - self._drag_offset)
                event.accept()
                return True
        if event_type == qevent_type("MouseButtonRelease") and self._dragging:
            self._dragging = False
            event.accept()
            return True
        return super().eventFilter(obj, event)

    def moveEvent(self, event):
        super().moveEvent(event)
        self._save_settings()

    def _tick(self):
        if self.ros is not None:
            if not self.ros.ok():
                self.close()
                return
            self.ros.spin_once()
            self.ros.flush_pending()

        self.state.update()
        now = time.monotonic()
        if self.ros is not None and now >= self.next_publish_ts:
            self.ros.publish(self.state, self.mode)
            self.next_publish_ts = now + self.publish_period
        self._refresh_ui()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            event.accept()
            return
        key = qt_key_value(event.key())
        if key in self.key_map:
            self.state.set_key(self.key_map[key], True)
            event.accept()
            return
        if key == qt_key_value(qt_enum("Key_A", "Key")):
            self.set_mode("manual" if self.mode == "autonomous" else "autonomous")
            event.accept()
            return
        if key == qt_key_value(qt_enum("Key_Space", "Key")):
            self.state.stop()
            event.accept()
            return
        if key == qt_key_value(qt_enum("Key_R", "Key")):
            self.state.stop()
            self.state.reset_pending = True
            event.accept()
            return
        if key == qt_key_value(qt_enum("Key_Escape", "Key")):
            self.close()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            event.accept()
            return
        key = qt_key_value(event.key())
        if key in self.key_map:
            self.state.set_key(self.key_map[key], False)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def changeEvent(self, event):
        if event.type() == qevent_type("WindowDeactivate"):
            self.state.pressed_keys.clear()
        super().changeEvent(event)

    def closeEvent(self, event):
        if not self._closed:
            self._closed = True
            self._save_settings()
            self.timer.stop()
            self.state.stop()
            if self.ros is not None:
                self.ros.publish_neutral(self.mode)
                self.ros.shutdown()
            print("Exiting")
        event.accept()


def qevent_type(name):
    if hasattr(QtCore.QEvent, name):
        return getattr(QtCore.QEvent, name)
    if hasattr(QtCore.QEvent, "Type"):
        return getattr(QtCore.QEvent.Type, name)
    raise AttributeError(f"QEvent type '{name}' was not found")


class RosInterface:
    def __init__(self, args, ros_version, window):
        self.args = args
        self.ros_version = ros_version
        self.window = window
        self.rospy = None
        self.rclpy = None
        self.node = None
        self.pub_throttle = None
        self.pub_steering = None
        self.pub_reset = None
        self.pub_joy = None
        self.pub_deadman = None
        self._shutdown = False
        self._pending_lock = Lock()
        self._pending_image = None
        self._pending_speed = None
        self._pending_lap_time = None
        self._pending_last_lap = None
        self._pending_best_lap = None
        self._pending_collision_count = None
        self._last_collision_count = None
        self._init_ros()

    def _init_ros(self):
        if self.ros_version == 1:
            import rospy

            self.rospy = rospy
            rospy.init_node("autodrive_teleop")
            self.pub_throttle = rospy.Publisher(
                topic_join(self.args.namespace, "throttle_command"), Float32, queue_size=1
            )
            self.pub_steering = rospy.Publisher(
                topic_join(self.args.namespace, "steering_command"), Float32, queue_size=1
            )
            self.pub_reset = rospy.Publisher("/autodrive/reset_command", Bool, queue_size=1)
            self.pub_joy = rospy.Publisher("/vesc/joy", Joy, queue_size=10)
            self.pub_deadman = rospy.Publisher("/autodrive/deadman_switch", Bool, queue_size=1)
            rospy.Subscriber(self.args.camera_topic, Image, self._on_image, queue_size=1)
            rospy.Subscriber(self.args.speed_topic, Float32, self._on_speed, queue_size=1)
            rospy.Subscriber(self.args.lap_time_topic, Float32, self._on_lap_time, queue_size=1)
            rospy.Subscriber(self.args.last_lap_topic, Float32, self._on_last_lap, queue_size=1)
            rospy.Subscriber(self.args.best_lap_topic, Float32, self._on_best_lap, queue_size=1)
            rospy.Subscriber(self.args.collision_count_topic, Int32, self._on_collision_count, queue_size=1)
            return

        if self.ros_version == 2:
            import rclpy
            from rclpy.qos import QoSProfile

            self.rclpy = rclpy
            rclpy.init()
            self.node = rclpy.create_node("autodrive_teleop")
            qos = QoSProfile(depth=1)
            self.pub_throttle = self.node.create_publisher(
                Float32, topic_join(self.args.namespace, "throttle_command"), qos
            )
            self.pub_steering = self.node.create_publisher(
                Float32, topic_join(self.args.namespace, "steering_command"), qos
            )
            self.pub_reset = self.node.create_publisher(Bool, "/autodrive/reset_command", qos)
            self.pub_joy = self.node.create_publisher(Joy, "/joy", 10)
            self.pub_deadman = self.node.create_publisher(Bool, "/autodrive/deadman_switch", qos)
            self.node.create_subscription(Image, self.args.camera_topic, self._on_image, qos)
            self.node.create_subscription(Float32, self.args.speed_topic, self._on_speed, qos)
            self.node.create_subscription(Float32, self.args.lap_time_topic, self._on_lap_time, qos)
            self.node.create_subscription(Float32, self.args.last_lap_topic, self._on_last_lap, qos)
            self.node.create_subscription(Float32, self.args.best_lap_topic, self._on_best_lap, qos)
            self.node.create_subscription(Int32, self.args.collision_count_topic, self._on_collision_count, qos)
            return

        raise RuntimeError(f"Unsupported ROS version '{self.ros_version}'")

    def ok(self):
        if self._shutdown:
            return False
        if self.ros_version == 1:
            return not self.rospy.is_shutdown()
        return self.rclpy.ok()

    def spin_once(self):
        if self.ros_version == 2 and not self._shutdown:
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def flush_pending(self):
        with self._pending_lock:
            image = self._pending_image
            speed = self._pending_speed
            lap_time = self._pending_lap_time
            last_lap = self._pending_last_lap
            best_lap = self._pending_best_lap
            collision_count = self._pending_collision_count
            self._pending_image = None
            self._pending_speed = None
            self._pending_lap_time = None
            self._pending_last_lap = None
            self._pending_best_lap = None
            self._pending_collision_count = None

        if image is not None:
            self.window.set_camera_image(image)
        if speed is not None:
            self.window.set_speed(speed)
        if lap_time is not None:
            self.window.set_lap_time(lap_time)
        if last_lap is not None:
            self.window.set_last_lap(last_lap)
        if best_lap is not None:
            self.window.set_best_lap(best_lap)
        if collision_count is not None:
            if self._last_collision_count is not None and collision_count > self._last_collision_count:
                self.window.set_collision_count(collision_count)
            self._last_collision_count = collision_count

    def publish(self, state, mode):
        if self._shutdown:
            return
        deadman_msg = Bool()
        deadman_msg.data = mode == "autonomous"
        self.pub_deadman.publish(deadman_msg)

        if mode != "manual":
            state.reset_pending = False
            return

        if not self.args.f1tenth:
            throttle_msg = Float32()
            steering_msg = Float32()
            reset_msg = Bool()
            throttle_msg.data = float(state.throttle) * self.args.autodrive_throttle_scale
            steering_msg.data = float(state.steering) * self.args.autodrive_steering_scale
            reset_msg.data = bool(state.reset_pending)
            self.pub_throttle.publish(throttle_msg)
            self.pub_steering.publish(steering_msg)
            self.pub_reset.publish(reset_msg)
            state.reset_pending = False
            return

        joy_msg = Joy()
        if self.ros_version == 1:
            joy_msg.header.stamp = self.rospy.Time.now()
        else:
            joy_msg.header.stamp = self.node.get_clock().now().to_msg()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 11
        joy_msg.axes[1] = float(state.throttle)
        joy_msg.axes[3] = float(state.steering)
        joy_msg.buttons[4] = 1
        joy_msg.buttons[5] = 0
        self.pub_joy.publish(joy_msg)

    def publish_neutral(self, mode):
        neutral = TeleopState(self.args)
        self.publish(neutral, mode)

    def shutdown(self):
        if self._shutdown:
            return
        self._shutdown = True
        if self.ros_version == 2 and self.node is not None:
            self.node.destroy_node()
            self.rclpy.shutdown()

    def _on_image(self, msg):
        image = image_msg_to_qimage(msg)
        if image is not None:
            with self._pending_lock:
                self._pending_image = image

    def _on_speed(self, msg):
        with self._pending_lock:
            self._pending_speed = float(msg.data)

    def _on_lap_time(self, msg):
        with self._pending_lock:
            self._pending_lap_time = float(msg.data)

    def _on_last_lap(self, msg):
        with self._pending_lock:
            self._pending_last_lap = float(msg.data)

    def _on_best_lap(self, msg):
        with self._pending_lock:
            self._pending_best_lap = float(msg.data)

    def _on_collision_count(self, msg):
        with self._pending_lock:
            self._pending_collision_count = int(msg.data)


def image_msg_to_qimage(msg):
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    if width <= 0 or height <= 0 or step <= 0:
        return None

    encoding = msg.encoding.lower()
    data = bytes(msg.data)

    if encoding in ("rgb8", "8uc3"):
        return QtGui.QImage(data, width, height, step, QtGui.QImage.Format_RGB888).copy()
    if encoding == "bgr8":
        image = QtGui.QImage(data, width, height, step, QtGui.QImage.Format_RGB888)
        return image.rgbSwapped().copy()
    if encoding in ("rgba8", "bgra8"):
        fmt = getattr(QtGui.QImage, "Format_RGBA8888", QtGui.QImage.Format_ARGB32)
        image = QtGui.QImage(data, width, height, step, fmt)
        if encoding == "bgra8":
            image = image.rgbSwapped()
        return image.copy()
    if encoding in ("mono8", "8uc1"):
        fmt = getattr(QtGui.QImage, "Format_Grayscale8", QtGui.QImage.Format_Indexed8)
        return QtGui.QImage(data, width, height, step, fmt).copy()

    return None


def main():
    args = parse_args()
    args.mode_explicit = any(
        arg == "--f1tenth" or arg == "--mode" or arg.startswith("--mode=")
        for arg in sys.argv[1:]
    )
    if args.camera_topic is None:
        args.camera_topic = topic_join(args.namespace, "front_camera")
    if args.speed_topic is None:
        args.speed_topic = topic_join(args.namespace, "speed")
    if args.lap_time_topic is None:
        args.lap_time_topic = topic_join(args.namespace, "lap_time")
    if args.last_lap_topic is None:
        args.last_lap_topic = topic_join(args.namespace, "last_lap_time")
    if args.best_lap_topic is None:
        args.best_lap_topic = topic_join(args.namespace, "best_lap_time")
    if args.collision_count_topic is None:
        args.collision_count_topic = topic_join(args.namespace, "collision_count")

    ros_version = resolve_ros_version(args.ros)
    app = QtWidgets.QApplication([sys.argv[0]])
    window = AutoDriveTeleopWindow(args, ros_version)
    ros = RosInterface(args, ros_version, window)
    window.attach_ros(ros)

    signal.signal(signal.SIGINT, lambda *_: window.close())

    print("AutoDRIVE Qt Widgets teleop")
    print(f"Qt: {QT_BINDING}, ROS{ros_version}, mode: {args.mode}")
    if args.f1tenth:
        print(f"publishing controls: {'/vesc/joy' if ros_version == 1 else '/joy'}")
    else:
        print(
            "publishing controls: "
            f"{topic_join(args.namespace, 'steering_command')}, "
            f"{topic_join(args.namespace, 'throttle_command')}"
        )
    print(f"camera: {args.camera_topic}")
    print(f"speed: {args.speed_topic}")
    print(f"collision count: {args.collision_count_topic}")
    print("deadman switch: /autodrive/deadman_switch")
    print("keys: Up/Down throttle, Left/Right steer, SPACE stop, R reset, A mode")

    window.show()
    try:
        return qt_exec(app)
    finally:
        ros.shutdown()


if __name__ == "__main__":
    sys.exit(main())
