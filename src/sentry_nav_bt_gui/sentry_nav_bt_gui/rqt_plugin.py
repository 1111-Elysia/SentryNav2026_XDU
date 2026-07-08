import os

from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from rqt_gui_py.plugin import Plugin

from sentry_nav_bt_gui.model import PARAMETER_NAMES, config_values, load_waypoints


POSTURES = (
    (1, "1 - 攻击姿态"),
    (2, "2 - 防御姿态"),
    (3, "3 - 移动姿态"),
    (4, "4 - 强化攻击姿态"),
    (5, "5 - 强化防御姿态"),
    (6, "6 - 强化移动姿态"),
)


class SimpleNavControlPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("SimpleNavControlPlugin")
        self._node = context.node
        self._target_node = "/sentry_nav_bt_test"
        self._waypoints = self._read_waypoints()
        self._loading_ui = False
        self._pending_get = None
        self._pending_set = None
        self._initial_read_requested = False

        self._get_client = self._node.create_client(
            GetParameters, self._target_node + "/get_parameters"
        )
        self._set_client = self._node.create_client(
            SetParametersAtomically,
            self._target_node + "/set_parameters_atomically",
        )

        self._widget = QWidget()
        self._widget.setObjectName("SimpleNavControlWidget")
        self._widget.setWindowTitle("单点导航控制")
        self._build_ui()
        context.add_widget(self._widget)

        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._poll_ros)
        self._timer.start(200)

    def _read_waypoints(self):
        waypoint_path = os.path.join(
            get_package_share_directory("sentry_nav_bt_test"),
            "config",
            "waypoints.json",
        )
        try:
            return load_waypoints(waypoint_path)
        except Exception as error:
            self._node.get_logger().error(f"加载点位失败: {error}")
            return {"init": (0.0, 0.0, 0.0)}

    def _build_ui(self):
        root = QVBoxLayout(self._widget)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("节点状态"))
        self._connection_label = QLabel("未连接")
        self._connection_label.setStyleSheet("font-weight: 600; color: #a33;")
        status_row.addWidget(self._connection_label)
        status_row.addStretch(1)
        root.addLayout(status_row)

        target_group = QGroupBox("目标点")
        target_form = QFormLayout(target_group)
        self._goal_combo = QComboBox()
        self._goal_combo.addItems(sorted(self._waypoints))
        if "init" in self._waypoints:
            self._goal_combo.setCurrentText("init")
        self._custom_pose = QCheckBox("使用临时坐标")
        target_form.addRow("点位", self._goal_combo)
        target_form.addRow("", self._custom_pose)

        coordinate_row = QHBoxLayout()
        self._goal_x = self._coordinate_spin()
        self._goal_y = self._coordinate_spin()
        self._goal_yaw = self._coordinate_spin()
        coordinate_row.addWidget(QLabel("X"))
        coordinate_row.addWidget(self._goal_x)
        coordinate_row.addWidget(QLabel("Y"))
        coordinate_row.addWidget(self._goal_y)
        coordinate_row.addWidget(QLabel("Yaw"))
        coordinate_row.addWidget(self._goal_yaw)
        target_form.addRow("坐标", coordinate_row)
        root.addWidget(target_group)

        posture_group = QGroupBox("哨兵姿态")
        posture_form = QFormLayout(posture_group)
        self._move_posture = self._posture_combo(3)
        self._wait_posture = self._posture_combo(1)
        posture_form.addRow("移动", self._move_posture)
        posture_form.addRow("等待", self._wait_posture)
        root.addWidget(posture_group)

        navigation_group = QGroupBox("导航")
        navigation_form = QFormLayout(navigation_group)
        self._controller = QComboBox()
        self._controller.setEditable(True)
        self._controller.addItems(["FollowPath", "HanBao"])
        self._reach_threshold = QDoubleSpinBox()
        self._reach_threshold.setRange(0.01, 10.0)
        self._reach_threshold.setDecimals(2)
        self._reach_threshold.setSingleStep(0.05)
        self._reach_threshold.setSuffix(" m")
        self._reach_threshold.setValue(0.25)
        self._wait_threshold = QDoubleSpinBox()
        self._wait_threshold.setRange(0.0, 10000.0)
        self._wait_threshold.setDecimals(1)
        self._wait_threshold.setSuffix(" s")
        self._wait_threshold.setValue(5.0)
        navigation_form.addRow("Controller", self._controller)
        navigation_form.addRow("到点距离", self._reach_threshold)
        navigation_form.addRow("时间阈值", self._wait_threshold)
        root.addWidget(navigation_group)

        action_row = QHBoxLayout()
        self._result_label = QLabel("等待连接行为树节点")
        self._result_label.setWordWrap(True)
        action_row.addWidget(self._result_label, 1)
        self._apply_button = QPushButton("应用配置")
        self._apply_button.setEnabled(False)
        self._apply_button.clicked.connect(self._apply_config)
        action_row.addWidget(self._apply_button)
        root.addLayout(action_row)

        self._goal_combo.currentTextChanged.connect(self._goal_selected)
        self._custom_pose.toggled.connect(self._update_coordinate_state)
        for spin in (self._goal_x, self._goal_y, self._goal_yaw):
            spin.valueChanged.connect(self._coordinate_edited)
        self._goal_selected(self._goal_combo.currentText())
        self._update_coordinate_state(False)

    @staticmethod
    def _coordinate_spin():
        spin = QDoubleSpinBox()
        spin.setRange(-1000.0, 1000.0)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        return spin

    @staticmethod
    def _posture_combo(default_value):
        combo = QComboBox()
        for value, label in POSTURES:
            combo.addItem(label, value)
        combo.setCurrentIndex(default_value - 1)
        return combo

    def _goal_selected(self, name):
        if not name or name not in self._waypoints:
            return
        if not self._loading_ui:
            self._custom_pose.setChecked(False)
        self._loading_ui = True
        x, y, yaw = self._waypoints[name]
        self._goal_x.setValue(x)
        self._goal_y.setValue(y)
        self._goal_yaw.setValue(yaw)
        self._loading_ui = False

    def _coordinate_edited(self, _value):
        if not self._loading_ui:
            self._custom_pose.setChecked(True)

    def _update_coordinate_state(self, custom):
        for spin in (self._goal_x, self._goal_y, self._goal_yaw):
            spin.setEnabled(custom)

    def _services_ready(self):
        return self._get_client.service_is_ready() and self._set_client.service_is_ready()

    def _poll_ros(self):
        connected = self._services_ready()
        self._apply_button.setEnabled(connected and self._pending_set is None)
        self._connection_label.setText("已连接" if connected else "未连接")
        self._connection_label.setStyleSheet(
            "font-weight: 600; color: #287a3d;" if connected
            else "font-weight: 600; color: #a33;"
        )

        if connected and not self._initial_read_requested:
            request = GetParameters.Request()
            request.names = list(PARAMETER_NAMES)
            self._pending_get = self._get_client.call_async(request)
            self._initial_read_requested = True

        if self._pending_get is not None and self._pending_get.done():
            try:
                self._load_parameter_response(self._pending_get.result())
                self._result_label.setText("已读取当前配置")
            except Exception as error:
                self._result_label.setText(f"读取配置失败：{error}")
                self._initial_read_requested = False
            self._pending_get = None

        if self._pending_set is not None and self._pending_set.done():
            try:
                response = self._pending_set.result()
                if response.result.successful:
                    self._result_label.setText("配置已生效")
                else:
                    self._result_label.setText(f"配置被拒绝：{response.result.reason}")
            except Exception as error:
                self._result_label.setText(f"提交失败：{error}")
            self._pending_set = None

    def _load_parameter_response(self, response):
        values = {}
        for name, parameter_value in zip(PARAMETER_NAMES, response.values):
            message = ParameterMessage(name=name, value=parameter_value)
            values[name] = Parameter.from_parameter_msg(message).value

        self._loading_ui = True
        goal_name = values["runtime_goal_name"]
        if self._goal_combo.findText(goal_name) < 0:
            self._goal_combo.addItem(goal_name)
        self._goal_combo.setCurrentText(goal_name)
        self._custom_pose.setChecked(values["runtime_use_custom_pose"])
        self._goal_x.setValue(values["runtime_goal_x"])
        self._goal_y.setValue(values["runtime_goal_y"])
        self._goal_yaw.setValue(values["runtime_goal_yaw"])
        self._move_posture.setCurrentIndex(int(values["runtime_move_posture"]) - 1)
        self._wait_posture.setCurrentIndex(int(values["runtime_wait_posture"]) - 1)
        self._controller.setCurrentText(values["runtime_controller"])
        self._reach_threshold.setValue(values["runtime_reach_threshold"])
        self._wait_threshold.setValue(values["runtime_wait_time_threshold"])
        self._loading_ui = False
        self._update_coordinate_state(self._custom_pose.isChecked())

    def _apply_config(self):
        values = config_values(
            self._goal_combo.currentText(),
            self._custom_pose.isChecked(),
            self._goal_x.value(),
            self._goal_y.value(),
            self._goal_yaw.value(),
            self._move_posture.currentData(),
            self._wait_posture.currentData(),
            self._controller.currentText(),
            self._reach_threshold.value(),
            self._wait_threshold.value(),
        )
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name=name, value=values[name]).to_parameter_msg()
            for name in PARAMETER_NAMES
        ]
        self._result_label.setText("正在应用配置...")
        self._pending_set = self._set_client.call_async(request)
        self._apply_button.setEnabled(False)

    def shutdown_plugin(self):
        self._timer.stop()
        self._node.destroy_client(self._get_client)
        self._node.destroy_client(self._set_client)

    def save_settings(self, plugin_settings, instance_settings):
        del plugin_settings, instance_settings

    def restore_settings(self, plugin_settings, instance_settings):
        del plugin_settings, instance_settings
