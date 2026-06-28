#!/usr/bin/env python3

import sys
import math
from os import path

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
from rm_referee_mock.widget_confirmed_line_edit import ConfirmedLineEdit
from rm_referee_mock.sentry_posture import POSTURE_NAMES, SentryPostureState

from ament_index_python.packages import get_package_share_directory

PACKAGE_SHARE = get_package_share_directory("rm_referee_mock")


class MatchControlWidget(QtWidgets.QWidget):
    DEFAULT_MATCH_REMAIN_SECONDS = 420
    DEFAULT_CHASSIS_POWER_LIMIT = 120
    DEFAULT_BUFFER_ENERGY_LIMIT = 60
    DEFAULT_SUPPLY_POINTS = (
        ("supply_point", -0.87, -5.04),
        ("supply_point_2", -1.35, -6.63),
    )
    SUPPLY_DETECT_RADIUS_M = 0.35
    SUPPLY_HEAL_HP_PER_SECOND = 100
    SUPPLY_PROJECTILE_PER_MINUTE = 100
    SUPPLY_PROJECTILE_MAX_MINUTES = 6
    GOLD_INITIAL_FLOOR = 400
    GOLD_TIMED_INCOME = (
        ("05:59", 359, 50),
        ("04:59", 299, 50),
        ("03:59", 239, 50),
        ("02:59", 179, 50),
        ("01:59", 119, 50),
        ("00:59", 59, 150),
    )
    RUNE_ACTIVATING_WINDOW_SECONDS = 20
    PRE_MATCH_STAGE_DURATIONS = {2: 15, 3: 5}
    MATCH_STAGE_INDEX = 4
    FINISHED_STAGE_INDEX = 5
    RMUL_GAME_TYPES = {4, 5}

    def __init__(self):
        super().__init__()

        ui_file = path.join(PACKAGE_SHARE, "assets", "match_control.ui")
        uic.loadUi(ui_file, self)
        self.setWindowTitle("Match Control")

        self.stage_countdown_remaining = 0
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self.update_time)
        self.hurt_burst_timer = QTimer(self)
        self.hurt_burst_timer.setInterval(500)
        self.hurt_burst_timer.timeout.connect(self._emit_hurt_burst)
        self.hurt_burst_remaining = 0
        self.sentry_posture = SentryPostureState(cooldown_seconds=5.0)
        self._suppress_posture_ui_change = False

        self.current_hurt_armor_id = 0
        self.trigger_hurt = False
        self.sentry_can_activate_flag = False
        self.small_rune_chances = 0
        self.big_rune_chances = 0
        self.last_remain_seconds = -1
        self.rune_window_timers = {"small": 0, "big": 0}
        self._suppress_rune_status_change = False
        self._last_sentry_hp = None
        self.resurrection_active = False
        self.resurrection_required_seconds = 0
        self.resurrection_progress = 0
        self.buy_revive_count = 0
        self.sentry_weak_active = False
        self.sentry_invincible_remaining_seconds = 0
        self.chassis_power_boost_remaining_seconds = 0
        self.shooter_17mm_heat = 0
        self.shooter_42mm_heat = 0
        self.buffer_energy = self.DEFAULT_BUFFER_ENERGY_LIMIT
        self.auto_supply_points = list(self.DEFAULT_SUPPLY_POINTS)
        self.auto_supply_pose = None
        self.auto_supply_zone_detected = False
        self.auto_supply_zone_name = ""
        self.supply_projectile_claimed = 0
        self.gold_income_claimed = set()

        self.init_ui()

    def init_ui(self):
        """初始化 UI 控件的默认值和选项"""
        self.resize(1310, 720)
        self.setMinimumSize(1310, 720)

        # 设置比赛类型选项 (索引0不使用，从1开始)
        self.comboBox_game_type.addItems([
            "[0] 未定义",
            "[1] RoboMaster 机甲大师超级对抗赛",
            "[2] RoboMaster 机甲大师高校单项赛",
            "[3] ICRA RoboMaster高校人工智能挑战赛",
            "[4] RoboMaster机甲大师高校联盟赛3V3对抗",
            "[5] RoboMaster 机甲大师高校联盟赛步兵对抗"
        ])
        self.comboBox_game_type.setCurrentIndex(1)  # 默认选择RMUC

        # 设置比赛阶段选项
        self.comboBox_game_stage.addItems([
            "[0] 未开始比赛",
            "[1] 准备阶段",
            "[2] 十五秒裁判系统自检阶段",
            "[3] 五秒倒计时",
            "[4] 比赛中",
            "[5] 比赛结算中"
        ])

        # 设置能量机关状态选项
        # 0:未激活, 1:激活中, 2:已激活
        energy_status = ["未激活", "激活中", "已激活"]
        self.comboBox_small_rune.addItems(energy_status)  # 小能量机关
        self.comboBox_big_rune.addItems(energy_status)  # 大能量机关
        self.comboBox_small_rune.currentIndexChanged.connect(
            lambda index, rune_name="small": self._on_rune_status_changed(rune_name, index)
        )
        self.comboBox_big_rune.currentIndexChanged.connect(
            lambda index, rune_name="big": self._on_rune_status_changed(rune_name, index)
        )

        # 设置高地占领状态
        occupy_status = ["无占领", "己方占领", "敌方占领"]
        self.comboBox_central_highland.addItems(occupy_status)  # 己方中央高地

        # 设置增益点状态
        shared_buff_status = ["未被占领", "己方占领", "对方占领", "双方占领"]
        outpost_buff_status = ["未被占领", "己方占领", "对方占领"]
        self.comboBox_central_buff.addItems(shared_buff_status)  # 中心增益点
        self.comboBox_fortress_buff.addItems(shared_buff_status)  # 己方堡垒增益点
        self.comboBox_outpost_buff.addItems(outpost_buff_status)  # 己方前哨站增益点

        # 设置飞镖击中目标
        dart_targets = [
            "未击中",
            "前哨站",
            "基地固定目标",
            "基地随机固定目标",
            "基地随机移动目标",
            "基地末端移动目标",
        ]
        self.comboBox_dart_target.addItems(dart_targets)

        # 设置HP默认值
        self.spinBox_hp_1.setMaximum(200)    # 英雄   近战优先1级上限血量200 ，远程优先1级上限血量150
        self.spinBox_hp_2.setMaximum(250)  # 工程
        self.spinBox_hp_3.setMaximum(250)  # 步兵
        self.spinBox_hp_4.setMaximum(250)  # 步兵
        self.spinBox_hp_7.setMaximum(400)  # 哨兵
        self.spinBox_hp_outpost.setMaximum(1500)  # 前哨站
        self.spinBox_hp_base.setMaximum(5000)  # 基地

        # 设置初始HP值
        self.spinBox_hp_1.setValue(200)
        self.spinBox_hp_2.setValue(250)
        self.spinBox_hp_3.setValue(250)
        self.spinBox_hp_4.setValue(250)
        self.spinBox_hp_7.setValue(400)
        self.spinBox_hp_outpost.setValue(1500)
        self.spinBox_hp_base.setValue(5000)

        self.pushButton_playpause.clicked.connect(self.on_play_pause_clicked)
        self.comboBox_game_stage.currentIndexChanged.connect(self._on_game_stage_changed)

        if hasattr(self, 'btn_send_hurt'):
            self.btn_send_hurt.clicked.connect(self.on_hurt_button_clicked)

        self._replace_lineedit_with_confirmed("lineEdit_time", "7:00")
        self._replace_lineedit_with_confirmed("lineEdit_topic_prefix", "/rm_referee")
        self._replace_lineedit_with_confirmed("lineEdit_dart_time", "0:00")

        if not hasattr(self, 'spinBox_ammo'):
            self.spinBox_ammo = QtWidgets.QSpinBox()
        if not hasattr(self, 'spinBox_ammo_42mm'):
            self.spinBox_ammo_42mm = None
        if not hasattr(self, 'spinBox_gold_coin'):
            self.spinBox_gold_coin = None
        if not hasattr(self, 'spinBox_ammo_fortress'):
            self.spinBox_ammo_fortress = None
        if not hasattr(self, 'spinBox_robot_id'):
            self.spinBox_robot_id = QtWidgets.QSpinBox()
        self._setup_projectile_allowance_inputs()
        self._setup_sentry_info_inputs()
        self._setup_v2_game_hp_inputs()

        if hasattr(self, "checkBox"):
            self.checkBox.setChecked(False)
            self.checkBox.setEnabled(False)
            self.checkBox.setText("资源区(协议废弃)")
            self.checkBox.setToolTip("V1.3.0 中 event_data bit1 已保留，该控件不再参与编码")

        self.label_stage_countdown = QtWidgets.QLabel(self)
        self.label_stage_countdown.setObjectName("label_stage_countdown")
        self.label_stage_countdown.setGeometry(430, 515, 210, 20)
        self.label_stage_countdown.setStyleSheet("color: #666666;")
        self._on_game_stage_changed(self.comboBox_game_stage.currentIndex())

    def set_auto_supply_points(self, points):
        cleaned_points = []
        for item in points:
            if len(item) != 3:
                continue
            name, x, y = item
            cleaned_points.append((str(name), float(x), float(y)))

        if cleaned_points:
            self.auto_supply_points = cleaned_points
            self._refresh_auto_supply_status()

    def update_auto_supply_pose(self, x, y, source=""):
        self.auto_supply_pose = (float(x), float(y), str(source))
        self._refresh_auto_supply_status()

    def _refresh_auto_supply_status(self):
        previous_detected = self.auto_supply_zone_detected
        previous_name = self.auto_supply_zone_name
        self.auto_supply_zone_detected = False
        self.auto_supply_zone_name = ""

        if self.auto_supply_pose is None or not self.auto_supply_points:
            return

        robot_x, robot_y, source = self.auto_supply_pose
        nearest_name = ""
        nearest_distance = None
        for name, point_x, point_y in self.auto_supply_points:
            distance = math.hypot(robot_x - point_x, robot_y - point_y)
            if nearest_distance is None or distance < nearest_distance:
                nearest_name = name
                nearest_distance = distance

        if nearest_distance is not None and nearest_distance <= self.SUPPLY_DETECT_RADIUS_M:
            self.auto_supply_zone_detected = True
            self.auto_supply_zone_name = nearest_name

        if (
            self.auto_supply_zone_detected != previous_detected
            or self.auto_supply_zone_name != previous_name
        ):
            state = "进入" if self.auto_supply_zone_detected else "离开"
            target = self.auto_supply_zone_name or previous_name or "补给区"
            print(f"[Supply] {state}{target}，定位来源={source or '-'}")

    def _replace_lineedit_with_confirmed(self, object_name, default_text):
        """替换一个LineEdit为ConfirmedLineEdit"""
        old_widget = getattr(self, object_name, None)
        if old_widget is None:
            print(f"Warning: {object_name} not found")
            return

        # 获取父控件和布局信息
        parent = old_widget.parent()
        if parent is None:
            print(f"Warning: {object_name} has no parent")
            return

        # 创建新的ConfirmedLineEdit
        new_widget = ConfirmedLineEdit(parent)
        new_widget.setObjectName(object_name)
        new_widget.setText(default_text)
        new_widget.set_confirmed_text(default_text)

        # 复制样式和属性
        new_widget.setEnabled(old_widget.isEnabled())
        new_widget.setReadOnly(old_widget.isReadOnly())
        new_widget.setPlaceholderText(old_widget.placeholderText())
        new_widget.setSizePolicy(old_widget.sizePolicy())
        new_widget.setMinimumSize(old_widget.minimumSize())
        new_widget.setMaximumSize(old_widget.maximumSize())

        # 从布局中替换
        layout = parent.layout()
        if layout:
            for i in range(layout.count()):
                if layout.itemAt(i).widget() == old_widget:
                    layout.removeWidget(old_widget)
                    layout.insertWidget(i, new_widget)
                    break
        else:
            # 如果没有布局，使用geometry
            new_widget.setGeometry(old_widget.geometry())

        # 隐藏并删除旧控件
        old_widget.hide()
        old_widget.deleteLater()

        # 更新引用
        setattr(self, object_name, new_widget)
        new_widget.show()

    def _ensure_labeled_spinbox(self, parent_widget, layout, insert_index, label_name, label_text, spinbox_name,
                                maximum=65535, default_value=0):
        spinbox = getattr(self, spinbox_name, None)
        if spinbox is None:
            label = QtWidgets.QLabel(parent_widget)
            label.setObjectName(label_name)
            label.setText(label_text)
            spinbox = QtWidgets.QSpinBox(parent_widget)
            spinbox.setObjectName(spinbox_name)
            spinbox.setValue(default_value)
            layout.insertWidget(insert_index, label)
            layout.insertWidget(insert_index + 1, spinbox)
            setattr(self, label_name, label)
            setattr(self, spinbox_name, spinbox)

        spinbox.setMaximum(maximum)
        return spinbox

    def _move_widget_to_layout(self, widget, target_layout):
        if widget is None:
            return

        current_parent = widget.parent()
        current_layout = current_parent.layout() if current_parent else None
        if current_layout is not None:
            current_layout.removeWidget(widget)

        target_parent = target_layout.parentWidget()
        if widget.parent() is not target_parent:
            widget.setParent(target_parent)

        if target_layout.indexOf(widget) == -1:
            target_layout.addWidget(widget)
        widget.show()

    def _ensure_projectile_allowance_group(self):
        group = getattr(self, "groupBox_projectile_allowance", None)
        layout = getattr(self, "verticalLayout_projectile_allowance", None)
        if group is None:
            group = QtWidgets.QGroupBox(self)
            group.setObjectName("groupBox_projectile_allowance")
            group.setTitle("ProjectileAllowance")
            setattr(self, "groupBox_projectile_allowance", group)

        if layout is None:
            layout = group.layout()
            if layout is None:
                layout = QtWidgets.QVBoxLayout(group)
            layout.setObjectName("verticalLayout_projectile_allowance")
            setattr(self, "verticalLayout_projectile_allowance", layout)

        return group, layout

    def _setup_projectile_allowance_inputs(self):
        self.spinBox_ammo.setMaximum(65535)

        if not hasattr(self, "verticalLayout_rs") or not hasattr(self, "groupBox_robot_status"):
            return

        projectile_group, projectile_layout = self._ensure_projectile_allowance_group()

        if not hasattr(self, "label_ammo"):
            self.label_ammo = QtWidgets.QLabel(projectile_group)
            self.label_ammo.setObjectName("label_ammo")
            self.label_ammo.setText("17mm弹丸")
        self._move_widget_to_layout(self.label_ammo, projectile_layout)
        self._move_widget_to_layout(self.spinBox_ammo, projectile_layout)

        self.spinBox_ammo_42mm = self._ensure_labeled_spinbox(
            projectile_group, projectile_layout, 2, "label_ammo_42mm", "42mm弹丸", "spinBox_ammo_42mm")
        self.spinBox_gold_coin = self._ensure_labeled_spinbox(
            projectile_group, projectile_layout, 4, "label_gold_coin", "剩余金币", "spinBox_gold_coin")
        self.spinBox_ammo_fortress = self._ensure_labeled_spinbox(
            projectile_group, projectile_layout, 6, "label_ammo_fortress", "堡垒17mm", "spinBox_ammo_fortress")

        self.groupBox_robot_status.setTitle("RobotStatus")
        if not hasattr(self, "checkBox_shooter_output"):
            self.checkBox_shooter_output = QtWidgets.QCheckBox(self.groupBox_robot_status)
            self.checkBox_shooter_output.setObjectName("checkBox_shooter_output")
        self.checkBox_shooter_output.setText("发射输出")
        self.checkBox_shooter_output.setToolTip(
            "发布 RobotStatus.power_management_shooter_output；取消勾选发布 0")
        self.checkBox_shooter_output.setChecked(True)
        self._move_widget_to_layout(self.checkBox_shooter_output, self.verticalLayout_rs)

        self.groupBox_robot_status.setGeometry(650, 20, 120, 120)
        projectile_group.setGeometry(650, 150, 120, 180)
        if hasattr(self, "groupBox_hurt"):
            self.groupBox_hurt.setGeometry(650, 340, 120, 120)
        if hasattr(self, "groupBox_service_echo"):
            self.groupBox_service_echo.setGeometry(650, 460, 120, 160)

    def _create_sentry_spinbox(self, maximum):
        spinbox = QtWidgets.QSpinBox(self.groupBox_sentry_detail)
        spinbox.setMaximum(maximum)
        spinbox.setValue(0)
        return spinbox

    def _setup_sentry_info_inputs(self):
        if not hasattr(self, "groupBox_service_echo") or not hasattr(self, "verticalLayout_echo"):
            return

        self.groupBox_service_echo.setTitle("Sentry Echo")
        self.label_echo_title.setText("当前姿态:")
        self.label_sentry_mode.setText("姿态: -")
        self.checkBox_can_activate_rune.setText("手动强制可打符")
        self.checkBox_can_activate_rune.setChecked(False)
        self.checkBox_can_activate_rune.setToolTip("用于调试时手动将 sentry_info_2 bit14 置 1")

        self.label_sentry_auto_activate = QtWidgets.QLabel(self.groupBox_service_echo)
        self.label_sentry_auto_activate.setObjectName("label_sentry_auto_activate")
        self.verticalLayout_echo.insertWidget(2, self.label_sentry_auto_activate)

        self.label_rune_window_status = QtWidgets.QLabel(self.groupBox_service_echo)
        self.label_rune_window_status.setObjectName("label_rune_window_status")
        self.verticalLayout_echo.insertWidget(3, self.label_rune_window_status)

        self.groupBox_sentry_detail = QtWidgets.QGroupBox(self)
        self.groupBox_sentry_detail.setObjectName("groupBox_sentry_detail")
        self.groupBox_sentry_detail.setTitle("SentryInfo 0x020D")
        self.groupBox_sentry_detail.setGeometry(780, 20, 230, 330)

        form_layout = QtWidgets.QFormLayout(self.groupBox_sentry_detail)
        form_layout.setObjectName("formLayout_sentry_detail")

        self.spinBox_sentry_exchange_projectile = self._create_sentry_spinbox(2047)
        self.spinBox_sentry_remote_projectile_exchange = self._create_sentry_spinbox(15)
        self.spinBox_sentry_remote_hp_exchange = self._create_sentry_spinbox(15)
        self.checkBox_sentry_can_free_revive = QtWidgets.QCheckBox(self.groupBox_sentry_detail)
        self.checkBox_sentry_can_buy_revive = QtWidgets.QCheckBox(self.groupBox_sentry_detail)
        self.spinBox_sentry_buy_revive_cost = self._create_sentry_spinbox(1023)
        self.checkBox_sentry_is_disengaged = QtWidgets.QCheckBox(self.groupBox_sentry_detail)
        self.spinBox_sentry_team_projectile_exchange = self._create_sentry_spinbox(2047)

        form_layout.addRow("成功补弹", self.spinBox_sentry_exchange_projectile)
        form_layout.addRow("远程补弹次数", self.spinBox_sentry_remote_projectile_exchange)
        form_layout.addRow("远程回血次数", self.spinBox_sentry_remote_hp_exchange)
        form_layout.addRow("可免费复活", self.checkBox_sentry_can_free_revive)
        form_layout.addRow("可金币复活", self.checkBox_sentry_can_buy_revive)
        form_layout.addRow("金币复活花费", self.spinBox_sentry_buy_revive_cost)
        form_layout.addRow("当前脱战", self.checkBox_sentry_is_disengaged)
        form_layout.addRow("队伍剩余可兑17mm", self.spinBox_sentry_team_projectile_exchange)

        self.label_resurrection_status = QtWidgets.QLabel(self.groupBox_sentry_detail)
        self.label_resurrection_status.setObjectName("label_resurrection_status")
        self.label_resurrection_status.setWordWrap(True)
        form_layout.addRow("复活读条", self.label_resurrection_status)

        self._update_resurrection_label()
        self._update_sentry_auto_activate_label()
        self._update_rune_window_label()

        self.groupBox_posture = QtWidgets.QGroupBox(self)
        self.groupBox_posture.setObjectName("groupBox_posture")
        self.groupBox_posture.setTitle("V2.0.0 哨兵姿态")
        self.groupBox_posture.setGeometry(1020, 20, 280, 465)
        posture_layout = QtWidgets.QFormLayout(self.groupBox_posture)

        self.comboBox_sentry_posture = QtWidgets.QComboBox(self.groupBox_posture)
        for mode in range(1, 7):
            self.comboBox_sentry_posture.addItem(f"[{mode}] {POSTURE_NAMES[mode]}", mode)
        self.pushButton_force_posture = QtWidgets.QPushButton("强制设置", self.groupBox_posture)
        posture_select_layout = QtWidgets.QHBoxLayout()
        posture_select_layout.addWidget(self.comboBox_sentry_posture)
        posture_select_layout.addWidget(self.pushButton_force_posture)
        posture_layout.addRow("当前姿态", posture_select_layout)

        self.posture_normal_spinboxes = {}
        self.posture_enhanced_spinboxes = {}
        labels = {1: "进攻", 2: "防御", 3: "移动"}
        for base in (1, 2, 3):
            spinbox = self._create_posture_time_spinbox(180, 180)
            self.posture_normal_spinboxes[base] = spinbox
            posture_layout.addRow(f"普通{labels[base]}剩余", spinbox)
            spinbox.valueChanged.connect(
                lambda value, posture=base: self._on_posture_time_changed(
                    "normal", posture, value))

        for base in (1, 2, 3):
            spinbox = self._create_posture_time_spinbox(15, 15)
            self.posture_enhanced_spinboxes[base] = spinbox
            posture_layout.addRow(f"强化{labels[base]}剩余", spinbox)
            spinbox.valueChanged.connect(
                lambda value, posture=base: self._on_posture_time_changed(
                    "enhanced", posture, value))

        self.label_posture_cooldown = QtWidgets.QLabel(self.groupBox_posture)
        self.label_posture_result = QtWidgets.QLabel(self.groupBox_posture)
        self.label_posture_result.setWordWrap(True)
        posture_layout.addRow("切换冷却", self.label_posture_cooldown)
        posture_layout.addRow("最近结果", self.label_posture_result)

        self.pushButton_reset_posture = QtWidgets.QPushButton("重置 180/15", self.groupBox_posture)
        self.pushButton_expire_posture = QtWidgets.QPushButton("耗尽当前强化", self.groupBox_posture)
        posture_button_layout = QtWidgets.QHBoxLayout()
        posture_button_layout.addWidget(self.pushButton_reset_posture)
        posture_button_layout.addWidget(self.pushButton_expire_posture)
        posture_layout.addRow(posture_button_layout)

        self.pushButton_enhanced_attack_2s = QtWidgets.QPushButton(
            "场景：强化进攻剩余 2s", self.groupBox_posture)
        posture_layout.addRow(self.pushButton_enhanced_attack_2s)

        self.pushButton_force_posture.clicked.connect(self._on_force_posture_clicked)
        self.pushButton_reset_posture.clicked.connect(self._on_reset_posture_clicked)
        self.pushButton_expire_posture.clicked.connect(self._on_expire_posture_clicked)
        self.pushButton_enhanced_attack_2s.clicked.connect(
            self._on_enhanced_attack_2s_clicked)
        self._sync_posture_ui()

    def _setup_v2_game_hp_inputs(self):
        self.groupBox_v2_game_hp = QtWidgets.QGroupBox(self)
        self.groupBox_v2_game_hp.setObjectName("groupBox_v2_game_hp")
        self.groupBox_v2_game_hp.setTitle("V2.0.0 对局血量")
        self.groupBox_v2_game_hp.setGeometry(780, 360, 230, 345)
        layout = QtWidgets.QFormLayout(self.groupBox_v2_game_hp)

        self.spinBox_damage_difference = QtWidgets.QSpinBox(self.groupBox_v2_game_hp)
        self.spinBox_damage_difference.setRange(-32768, 32767)
        self.spinBox_enemy_outpost_hp = QtWidgets.QSpinBox(self.groupBox_v2_game_hp)
        self.spinBox_enemy_outpost_hp.setRange(0, 5000)
        self.spinBox_enemy_outpost_hp.setValue(1500)
        self.spinBox_enemy_base_hp = QtWidgets.QSpinBox(self.groupBox_v2_game_hp)
        self.spinBox_enemy_base_hp.setRange(0, 10000)
        self.spinBox_enemy_base_hp.setValue(5000)
        layout.addRow("伤害差", self.spinBox_damage_difference)
        layout.addRow("敌方前哨站", self.spinBox_enemy_outpost_hp)
        layout.addRow("敌方基地", self.spinBox_enemy_base_hp)

        self.spinBox_test_shooter_heat = QtWidgets.QSpinBox(self.groupBox_v2_game_hp)
        self.spinBox_test_shooter_heat.setRange(0, 1000)
        self.spinBox_test_shooter_heat.valueChanged.connect(
            lambda value: setattr(self, "shooter_17mm_heat", int(value)))
        layout.addRow("17mm 热量", self.spinBox_test_shooter_heat)

        self.pushButton_destroy_enemy_outpost = QtWidgets.QPushButton(
            "敌方前哨站归零", self.groupBox_v2_game_hp)
        self.pushButton_reset_enemy_hp = QtWidgets.QPushButton(
            "重置敌方建筑", self.groupBox_v2_game_hp)
        layout.addRow(self.pushButton_destroy_enemy_outpost)
        layout.addRow(self.pushButton_reset_enemy_hp)
        self.pushButton_heat_shot = QtWidgets.QPushButton("模拟射击：热量 +10", self.groupBox_v2_game_hp)
        self.pushButton_hurt_burst = QtWidgets.QPushButton("连续受击 3 次", self.groupBox_v2_game_hp)
        layout.addRow(self.pushButton_heat_shot)
        layout.addRow(self.pushButton_hurt_burst)
        self.pushButton_destroy_enemy_outpost.clicked.connect(
            lambda: self.spinBox_enemy_outpost_hp.setValue(0))
        self.pushButton_reset_enemy_hp.clicked.connect(self._reset_enemy_hp)
        self.pushButton_heat_shot.clicked.connect(self._simulate_heat_shot)
        self.pushButton_hurt_burst.clicked.connect(self._start_hurt_burst)

    def _create_posture_time_spinbox(self, maximum, value):
        spinbox = QtWidgets.QSpinBox(self.groupBox_posture)
        spinbox.setRange(0, maximum)
        spinbox.setSuffix(" s")
        spinbox.setValue(value)
        return spinbox

    def _on_posture_time_changed(self, timer_type, base, value):
        if self._suppress_posture_ui_change:
            return
        if timer_type == "normal":
            self.sentry_posture.set_normal_remaining(base, value)
        else:
            self.sentry_posture.set_enhanced_remaining(base, value)
            if (
                value == 0
                and self.sentry_posture.current_mode == base + 3
            ):
                self.sentry_posture.expire_current_enhanced()
        self._sync_posture_ui()

    def _sync_posture_ui(self):
        if not hasattr(self, "comboBox_sentry_posture"):
            return
        self._suppress_posture_ui_change = True
        try:
            self.comboBox_sentry_posture.setCurrentIndex(
                max(0, self.sentry_posture.current_mode - 1))
            for base, spinbox in self.posture_normal_spinboxes.items():
                spinbox.setValue(self.sentry_posture.normal_remaining[base])
            for base, spinbox in self.posture_enhanced_spinboxes.items():
                spinbox.setValue(self.sentry_posture.enhanced_remaining[base])
        finally:
            self._suppress_posture_ui_change = False

        remaining = self.sentry_posture.cooldown_remaining()
        self.label_posture_cooldown.setText(
            f"{remaining:.1f}s" if remaining > 0.0 else "就绪")
        self.label_posture_result.setText(self.sentry_posture.last_result)
        self.label_sentry_mode.setText(
            f"姿态: {POSTURE_NAMES[self.sentry_posture.current_mode]}")

    def _on_force_posture_clicked(self):
        mode = int(self.comboBox_sentry_posture.currentData())
        accepted, message = self.sentry_posture.force_mode(mode)
        print(f"[Posture-Debug] {'成功' if accepted else '拒绝'}: {message}")
        self._sync_posture_ui()

    def _on_reset_posture_clicked(self):
        self.sentry_posture.reset()
        print("[Posture-Debug] 已重置姿态和六个剩余时间")
        self._sync_posture_ui()

    def _on_expire_posture_clicked(self):
        changed = self.sentry_posture.expire_current_enhanced()
        print(f"[Posture-Debug] {self.sentry_posture.last_result}")
        if changed:
            self.update_sentry_echo(self.sentry_posture.current_mode)
        self._sync_posture_ui()

    def _on_enhanced_attack_2s_clicked(self):
        self.sentry_posture.set_enhanced_remaining(1, 2)
        self.sentry_posture.force_mode(4)
        print("[Posture-Debug] 场景已设置：强化进攻剩余 2s")
        self._sync_posture_ui()

    def _reset_enemy_hp(self):
        self.spinBox_enemy_outpost_hp.setValue(1500)
        self.spinBox_enemy_base_hp.setValue(5000)

    def _simulate_heat_shot(self):
        self.spinBox_test_shooter_heat.setValue(
            min(self.spinBox_test_shooter_heat.maximum(), self.shooter_17mm_heat + 10))

    def _start_hurt_burst(self):
        self.hurt_burst_remaining = 3
        self._emit_hurt_burst()
        self.hurt_burst_timer.start()

    def _emit_hurt_burst(self):
        if self.hurt_burst_remaining <= 0:
            self.hurt_burst_timer.stop()
            return
        self.current_hurt_armor_id = self.spinBox_hurt_armor_id.value()
        self.trigger_hurt = True
        self.hurt_burst_remaining -= 1

    def request_sentry_posture(self, mode):
        accepted, message = self.sentry_posture.request_mode(mode)
        if accepted:
            self.update_sentry_echo(self.sentry_posture.current_mode)
        self._sync_posture_ui()
        return accepted, message

    def _tick_sentry_posture(self):
        active = (
            self.comboBox_game_stage.currentIndex() == self.MATCH_STAGE_INDEX
            and self.spinBox_hp_7.value() > 0
        )
        fallback = self.sentry_posture.tick(active=active)
        if fallback:
            print(f"[Posture-Timeout] {self.sentry_posture.last_result}")
            self.update_sentry_echo(self.sentry_posture.current_mode)
        self._sync_posture_ui()

    def _get_rune_combo(self, rune_name):
        return self.comboBox_small_rune if rune_name == "small" else self.comboBox_big_rune

    def _get_rune_label(self, rune_name):
        return "小能量机关" if rune_name == "small" else "大能量机关"

    def _update_rune_combo_text(self, rune_name):
        combo = self._get_rune_combo(rune_name)
        remaining = self.rune_window_timers[rune_name]
        active_text = f"激活中({remaining}s)" if remaining > 0 else "激活中"
        combo.setItemText(0, "未激活")
        combo.setItemText(1, active_text)
        combo.setItemText(2, "已激活")

    def _set_rune_status(self, rune_name, ui_index):
        combo = self._get_rune_combo(rune_name)
        changed = combo.currentIndex() != ui_index
        self._suppress_rune_status_change = True
        try:
            if changed:
                combo.setCurrentIndex(ui_index)
        finally:
            self._suppress_rune_status_change = False

        self._apply_rune_status_change(rune_name, ui_index, initiated_by_user=False)

    def _apply_rune_status_change(self, rune_name, ui_index, initiated_by_user):
        if ui_index == 1:
            if self.rune_window_timers[rune_name] <= 0:
                self.rune_window_timers[rune_name] = self.RUNE_ACTIVATING_WINDOW_SECONDS
                print(
                    f"[Rune] {self._get_rune_label(rune_name)}进入正在激活状态，"
                    f"开启{self.RUNE_ACTIVATING_WINDOW_SECONDS}s打符窗口"
                )
        else:
            had_timer = self.rune_window_timers[rune_name] > 0
            self.rune_window_timers[rune_name] = 0
            if ui_index == 2 and had_timer:
                print(f"[Rune] {self._get_rune_label(rune_name)}已被成功激活")
            elif ui_index == 0 and initiated_by_user and had_timer:
                print(f"[Rune] {self._get_rune_label(rune_name)}被手动重置为未激活")

        self._refresh_sentry_activate_flag()
        self._update_rune_combo_text(rune_name)
        self._update_rune_window_label()

    def _on_rune_status_changed(self, rune_name, ui_index):
        if self._suppress_rune_status_change:
            return
        self._apply_rune_status_change(rune_name, ui_index, initiated_by_user=True)

    def _update_rune_window_label(self):
        if not hasattr(self, "label_rune_window_status"):
            return

        parts = []
        for rune_name in ("small", "big"):
            combo = self._get_rune_combo(rune_name)
            remaining = self.rune_window_timers[rune_name]
            self._update_rune_combo_text(rune_name)
            if combo.currentIndex() == 1 and remaining > 0:
                parts.append(f"{'小' if rune_name == 'small' else '大'}{remaining}s")

        text = "打符窗: " + " ".join(parts) if parts else "打符窗: -"
        self.label_rune_window_status.setText(text)

    def _is_rmul_game_type(self):
        return self.comboBox_game_type.currentIndex() in self.RMUL_GAME_TYPES

    def _reset_match_timer(self):
        self.lineEdit_time.set_confirmed_text(self._format_seconds(self.DEFAULT_MATCH_REMAIN_SECONDS))
        self.stage_countdown_remaining = 0
        self._update_stage_countdown_display()

    def _reset_match_runtime_state(self):
        self.sentry_can_activate_flag = False
        self.small_rune_chances = 0
        self.big_rune_chances = 0
        self.last_remain_seconds = -1
        self.rune_window_timers = {"small": 0, "big": 0}
        self.supply_projectile_claimed = 0
        self.gold_income_claimed = set()
        self.sentry_posture.reset()
        if hasattr(self, "spinBox_damage_difference"):
            self.spinBox_damage_difference.setValue(0)
            self._reset_enemy_hp()
        self._reset_resurrection_state(clear_buy_count=True)
        if hasattr(self, "checkBox_can_activate_rune"):
            self.checkBox_can_activate_rune.setChecked(False)
        self._update_sentry_auto_activate_label()
        self._update_rune_window_label()
        self._sync_posture_ui()

    def _refresh_sentry_activate_flag(self):
        current_small_status = self.comboBox_small_rune.currentIndex()
        current_big_status = self.comboBox_big_rune.currentIndex()
        self.sentry_can_activate_flag = (
            (self.small_rune_chances > 0 and current_small_status == 0)
            or (self.big_rune_chances > 0 and current_big_status == 0)
        )
        self._update_sentry_auto_activate_label()

    def _update_sentry_auto_activate_label(self):
        if hasattr(self, "label_sentry_auto_activate"):
            text = "是" if self.sentry_can_activate_flag else "否"
            self.label_sentry_auto_activate.setText(f"自动可打符: {text}")

    def _on_game_stage_changed(self, stage_index):
        if stage_index in self.PRE_MATCH_STAGE_DURATIONS:
            self.stage_countdown_remaining = self.PRE_MATCH_STAGE_DURATIONS[stage_index]
            self._update_stage_countdown_display()
            self._set_countdown_running(True)
            return

        self.stage_countdown_remaining = 0
        self._update_stage_countdown_display()
        if stage_index in (0, 1):
            self._reset_match_timer()
            self._reset_match_runtime_state()
            self._set_countdown_running(False)
            return

        if stage_index == self.MATCH_STAGE_INDEX:
            remain_seconds = self._parse_time_to_seconds()
            self._sync_gold_income(remain_seconds)
            self._check_rune_refresh(remain_seconds)
            self._set_countdown_running(True)
            return

        if stage_index == self.FINISHED_STAGE_INDEX:
            self._set_countdown_running(False)

    def _update_stage_countdown_display(self):
        if not hasattr(self, "label_stage_countdown"):
            return
        if self.stage_countdown_remaining > 0:
            self.label_stage_countdown.setText(
                f"预倒计时: {self._format_seconds(self.stage_countdown_remaining)}"
            )
        else:
            self.label_stage_countdown.clear()

    def _set_countdown_running(self, running):
        self.pushButton_playpause.setText("⏸" if running else "▶")
        self.lineEdit_time.setReadOnly(running)
        if running:
            self.countdown_timer.start(1000)
        else:
            self.countdown_timer.stop()

    def _tick_stage_countdown(self):
        stage_index = self.comboBox_game_stage.currentIndex()
        if self.stage_countdown_remaining <= 0:
            self.stage_countdown_remaining = self.PRE_MATCH_STAGE_DURATIONS[stage_index]

        self.stage_countdown_remaining -= 1
        self._update_stage_countdown_display()
        if self.stage_countdown_remaining > 0:
            return

        next_stage = 3 if stage_index == 2 else self.MATCH_STAGE_INDEX
        self.comboBox_game_stage.setCurrentIndex(next_stage)

    def _tick_match_countdown(self):
        total_seconds = self._parse_time_to_seconds()
        if total_seconds <= 0:
            if self.comboBox_game_stage.currentIndex() == self.MATCH_STAGE_INDEX:
                self.comboBox_game_stage.setCurrentIndex(self.FINISHED_STAGE_INDEX)
            self._set_countdown_running(False)
            return

        total_seconds -= 1
        self.lineEdit_time.set_confirmed_text(self._format_seconds(total_seconds))
        self._sync_gold_income(total_seconds)
        self._tick_supply_zone_effects(total_seconds)
        self._check_rune_refresh(total_seconds)

        if total_seconds == 0:
            if self.comboBox_game_stage.currentIndex() == self.MATCH_STAGE_INDEX:
                self.comboBox_game_stage.setCurrentIndex(self.FINISHED_STAGE_INDEX)
            self._set_countdown_running(False)

    def _format_seconds(self, total_seconds):
        minutes, seconds = divmod(max(0, total_seconds), 60)
        return f"{minutes}:{seconds:02d}"

    def _get_stage_remain_time(self):
        stage_index = self.comboBox_game_stage.currentIndex()
        if stage_index in self.PRE_MATCH_STAGE_DURATIONS and self.stage_countdown_remaining > 0:
            return self.stage_countdown_remaining
        return self._parse_time_to_seconds()

    def on_play_pause_clicked(self):
        """处理播放/暂停按钮点击事件"""
        if self.pushButton_playpause.text() == "▶":
            stage_index = self.comboBox_game_stage.currentIndex()
            if stage_index in self.PRE_MATCH_STAGE_DURATIONS and self.stage_countdown_remaining <= 0:
                self.stage_countdown_remaining = self.PRE_MATCH_STAGE_DURATIONS[stage_index]
                self._update_stage_countdown_display()
            if stage_index == self.MATCH_STAGE_INDEX:
                self._check_rune_refresh(self._parse_time_to_seconds())
            self._set_countdown_running(True)
        else:
            self._set_countdown_running(False)

    def on_hurt_button_clicked(self):
        """点击发送伤害"""
        self.trigger_hurt = True
        self.current_hurt_armor_id = self.spinBox_hurt_armor_id.value()
        
        
    def _reset_hurt_trigger(self):
        self.trigger_hurt = False

    def update_time(self):
        """更新赛前阶段和正式比赛倒计时"""
        self._sync_resurrection_state()
        if self.comboBox_game_stage.currentIndex() in self.PRE_MATCH_STAGE_DURATIONS:
            self._tick_stage_countdown()
            return

        self._tick_rune_windows()
        self._tick_sentry_posture()
        self._tick_resurrection()
        self._tick_match_countdown()

    def _tick_rune_windows(self):
        for rune_name in ("small", "big"):
            combo = self._get_rune_combo(rune_name)
            if combo.currentIndex() != 1:
                continue

            remaining = self.rune_window_timers[rune_name]
            if remaining <= 0:
                continue

            remaining -= 1
            self.rune_window_timers[rune_name] = remaining
            if remaining == 0:
                print(
                    f"[Timeout] {self._get_rune_label(rune_name)}20s打符窗口结束，"
                    "仍未成功激活，恢复为未激活"
                )
                self._set_rune_status(rune_name, 0)
                continue

        self._update_rune_window_label()

    def _parse_time_to_seconds(self, time_str=None):
        """解析时间字符串 MM:SS 转换为秒数"""
        # 如果没有提供时间字符串，使用已确认的值
        if time_str is None:
            time_str = self.lineEdit_time.get_confirmed_text()
        try:
            parts = time_str.strip().split(":")
            if len(parts) == 2:
                minutes = int(parts[0])
                seconds = int(parts[1])
                return minutes * 60 + seconds
        except (ValueError, IndexError):
            pass
        return 0

    def get_topic_prefix(self):
        """获取话题前缀"""
        prefix = self.lineEdit_topic_prefix.get_confirmed_text()
        # 确保前缀以/开头，但不以/结尾
        if prefix and not prefix.startswith("/"):
            prefix = "/" + prefix
        if prefix.endswith("/"):
            prefix = prefix.rstrip("/")
        return prefix if prefix else ""
    
    def get_event_data_value(self):
        """
            按 2026 V1.3.0 协议计算 EventData 位图值。
        """
        def map_rune_status(ui_index):
            if ui_index == 0:
                return 0
            if ui_index == 1:
                return 2
            if ui_index == 2:
                return 1
            return 0

        event_val = 0
        auto_supply = self.auto_supply_zone_detected and self.comboBox_game_stage.currentIndex() == self.MATCH_STAGE_INDEX

        event_val |= int(self.checkBox_2.isChecked() or auto_supply) << 0
        if self._is_rmul_game_type():
            event_val |= int(self.checkBox_3.isChecked() or auto_supply) << 2

        event_val |= (map_rune_status(self.comboBox_small_rune.currentIndex()) & 0x03) << 3
        event_val |= (map_rune_status(self.comboBox_big_rune.currentIndex()) & 0x03) << 5
        event_val |= (self.comboBox_central_highland.currentIndex() & 0x03) << 7
        event_val |= int(self.checkBox_4.isChecked()) << 9

        dart_hit_time = min(420, max(0, self._parse_time_to_seconds(self.lineEdit_dart_time.get_confirmed_text())))
        event_val |= (dart_hit_time & 0x01FF) << 11
        event_val |= (self.comboBox_dart_target.currentIndex() & 0x07) << 20

        if self._is_rmul_game_type():
            event_val |= (self.comboBox_central_buff.currentIndex() & 0x03) << 23
        event_val |= (self.comboBox_fortress_buff.currentIndex() & 0x03) << 25
        event_val |= (self.comboBox_outpost_buff.currentIndex() & 0x03) << 27
        event_val |= int(self.checkBox_base_buff.isChecked()) << 29

        return event_val

    def _manual_supply_zone_selected(self):
        supply_selected = self.checkBox_2.isChecked() if hasattr(self, "checkBox_2") else False
        rmul_supply_selected = self.checkBox_3.isChecked() if hasattr(self, "checkBox_3") else False
        return supply_selected or rmul_supply_selected

    def is_supply_zone_occupied(self):
        if self.comboBox_game_stage.currentIndex() != self.MATCH_STAGE_INDEX:
            return False
        return self._manual_supply_zone_selected() or self.auto_supply_zone_detected

    def get_rfid_status_value(self):
        rfid_status = 0
        rfid_status_2 = 0

        if hasattr(self, "checkBox_base_buff") and self.checkBox_base_buff.isChecked():
            rfid_status |= 1 << 0
        if hasattr(self, "comboBox_fortress_buff") and self.comboBox_fortress_buff.currentIndex() in (1, 3):
            rfid_status |= 1 << 17
        if hasattr(self, "comboBox_outpost_buff") and self.comboBox_outpost_buff.currentIndex() == 1:
            rfid_status |= 1 << 18
        if self.auto_supply_zone_detected or (hasattr(self, "checkBox_3") and self.checkBox_3.isChecked()):
            rfid_status |= 1 << 19
        if hasattr(self, "checkBox_2") and self.checkBox_2.isChecked():
            rfid_status |= 1 << 20

        return {
            "rfid_status": rfid_status,
            "rfid_status_2": rfid_status_2,
        }

    def _sync_gold_income(self, remain_seconds):
        if self.comboBox_game_stage.currentIndex() != self.MATCH_STAGE_INDEX or not self.spinBox_gold_coin:
            return

        if "initial" not in self.gold_income_claimed:
            current_gold = self.spinBox_gold_coin.value()
            if current_gold < self.GOLD_INITIAL_FLOOR:
                self.spinBox_gold_coin.setValue(self.GOLD_INITIAL_FLOOR)
                print(f"[Economy] 初始低保金币补足到 {self.GOLD_INITIAL_FLOOR}")
            self.gold_income_claimed.add("initial")

        for event_name, threshold_seconds, amount in self.GOLD_TIMED_INCOME:
            if remain_seconds > threshold_seconds or event_name in self.gold_income_claimed:
                continue
            self.spinBox_gold_coin.setValue(
                min(self.spinBox_gold_coin.maximum(), self.spinBox_gold_coin.value() + amount)
            )
            self.gold_income_claimed.add(event_name)
            print(f"[Economy] {event_name} 低保金币 +{amount}，当前 {self.spinBox_gold_coin.value()}")

    def _available_supply_projectile(self, remain_seconds):
        elapsed_seconds = self.DEFAULT_MATCH_REMAIN_SECONDS - max(0, min(self.DEFAULT_MATCH_REMAIN_SECONDS, remain_seconds))
        elapsed_minutes = min(self.SUPPLY_PROJECTILE_MAX_MINUTES, max(0, elapsed_seconds // 60))
        return elapsed_minutes * self.SUPPLY_PROJECTILE_PER_MINUTE

    def _claim_supply_projectile(self, remain_seconds):
        available = self._available_supply_projectile(remain_seconds)
        delta = max(0, available - self.supply_projectile_claimed)
        if delta <= 0:
            return

        self.spinBox_ammo.setValue(min(self.spinBox_ammo.maximum(), self.spinBox_ammo.value() + delta))
        self.supply_projectile_claimed += delta
        print(
            f"[Supply] 领取补给区累积发弹量 +{delta}，"
            f"已领取 {self.supply_projectile_claimed}，当前 17mm {self.spinBox_ammo.value()}"
        )

    def _tick_supply_zone_effects(self, remain_seconds):
        if not self.is_supply_zone_occupied():
            return

        current_hp = self.spinBox_hp_7.value()
        max_hp = self.spinBox_hp_7.maximum()
        if 0 < current_hp < max_hp:
            new_hp = min(max_hp, current_hp + self.SUPPLY_HEAL_HP_PER_SECOND)
            self.spinBox_hp_7.setValue(new_hp)
            print(f"[Supply] 补给区回血 +{new_hp - current_hp}，当前 HP {new_hp}/{max_hp}")

        self._claim_supply_projectile(remain_seconds)

    def get_game_status(self):
        """获取当前比赛状态"""
        self._sync_resurrection_state()
        sentry_info_status = self.get_sentry_info_status()
        status = {
            "game_type": self.comboBox_game_type.currentIndex(),
            "game_progress": self.comboBox_game_stage.currentIndex(),
            "stage_remain_time": self._get_stage_remain_time(),
            "robot_hp": {
                "hero": self.spinBox_hp_1.value(),
                "engineer": self.spinBox_hp_2.value(),
                "infantry_3": self.spinBox_hp_3.value(),
                "infantry_4": self.spinBox_hp_4.value(),
                "sentry": self.spinBox_hp_7.value(),
                "ally_outpost": self.spinBox_hp_outpost.value(),
                "ally_base": self.spinBox_hp_base.value(),
                "damage_difference": self.spinBox_damage_difference.value(),
                "enemy_outpost": self.spinBox_enemy_outpost_hp.value(),
                "enemy_base": self.spinBox_enemy_base_hp.value(),
            },
            # 详细机器人状态
            "robot_status": {
                "id": self.spinBox_robot_id.value(),
                "current_hp": self.spinBox_hp_7.value(), # 复用哨兵血量
                "max_hp": self.spinBox_hp_7.maximum(),
                "ammo": self.spinBox_ammo.value(),
                "chassis_power_limit": self._current_chassis_power_limit(),
                "shooter_output": self._current_shooter_output(),
            },
            "projectile_allowance": {
                "projectile_allowance_17mm": self.spinBox_ammo.value(),
                "projectile_allowance_42mm": self.spinBox_ammo_42mm.value() if self.spinBox_ammo_42mm else 0,
                "remaining_gold_coin": self.spinBox_gold_coin.value() if self.spinBox_gold_coin else 0,
                "projectile_allowance_fortress": self.spinBox_ammo_fortress.value()
                if self.spinBox_ammo_fortress else 0,
            },
            # 事件数据
            "event_data": self.get_event_data_value(),
            "rfid_status": self.get_rfid_status_value(),
            # 伤害数据
            "hurt_data": {
                "armor_id": self.current_hurt_armor_id if self.trigger_hurt else 0,
                # 如果 trigger 为 false, 发 0 或者不发(plugin处理)
                "trigger": self.trigger_hurt 
            },
            "sentry_can_activate": sentry_info_status["can_activate"],
            "sentry_info": sentry_info_status,
            "power_heat_data": {
                "buffer_energy": self.buffer_energy,
                "shooter_17mm_1_barrel_heat": self.shooter_17mm_heat,
                "shooter_42mm_barrel_heat": self.shooter_42mm_heat,
            },
        }
        if self.trigger_hurt:
            self.trigger_hurt = False

        return status

    def _current_shooter_output(self):
        manual_enabled = (
            self.checkBox_shooter_output.isChecked()
            if hasattr(self, "checkBox_shooter_output")
            else True
        )
        return manual_enabled and not self.sentry_weak_active

    def get_sentry_info_status(self):
        manual_can_activate = (
            self.checkBox_can_activate_rune.isChecked()
            if hasattr(self, "checkBox_can_activate_rune")
            else False
        )
        return {
            "exchange_projectile": self.spinBox_sentry_exchange_projectile.value(),
            "remote_projectile_exchange_count": self.spinBox_sentry_remote_projectile_exchange.value(),
            "remote_hp_exchange_count": self.spinBox_sentry_remote_hp_exchange.value(),
            "can_free_revive": self.checkBox_sentry_can_free_revive.isChecked(),
            "can_buy_revive": self.checkBox_sentry_can_buy_revive.isChecked(),
            "buy_revive_cost": self.spinBox_sentry_buy_revive_cost.value(),
            "is_disengaged": self.checkBox_sentry_is_disengaged.isChecked(),
            "team_projectile_exchange_remaining": self.spinBox_sentry_team_projectile_exchange.value(),
            "can_activate": self.sentry_can_activate_flag or manual_can_activate,
            "current_mode": self.sentry_posture.current_mode,
            "base_posture": (
                self.sentry_posture.current_mode - 3
                if self.sentry_posture.current_mode >= 4
                else self.sentry_posture.current_mode
            ),
            "enhanced": self.sentry_posture.current_mode >= 4,
            "posture_remaining": dict(self.sentry_posture.normal_remaining),
            "enhanced_posture_remaining": dict(self.sentry_posture.enhanced_remaining),
            "sentry_info_3": self.sentry_posture.pack_sentry_info_3(),
        }

    def update_sentry_echo(self, mode):
        """更新 UI 显示收到的指令"""
        if hasattr(self, 'label_sentry_mode'):
            # 简单映射模式名
            txt = POSTURE_NAMES.get(mode, str(mode))
            self.label_sentry_mode.setText(f"姿态: {txt}")
            # 可以变个色提示更新
            self.label_sentry_mode.setStyleSheet("color: red; font-weight: bold;")
            QTimer.singleShot(500, lambda: self.label_sentry_mode.setStyleSheet("color: blue; font-weight: bold;"))    

    def _round_half_up(self, value):
        return int(math.floor(value + 0.5))

    def _calculate_resurrection_required_seconds(self, remain_seconds):
        remain_seconds = min(self.DEFAULT_MATCH_REMAIN_SECONDS, max(0, int(remain_seconds)))
        base_seconds = 10 + ((self.DEFAULT_MATCH_REMAIN_SECONDS - remain_seconds) / 10.0)
        return self._round_half_up(base_seconds + 20 * self.buy_revive_count)

    def _reset_power_heat_after_death(self):
        self.shooter_17mm_heat = 0
        self.shooter_42mm_heat = 0
        self.buffer_energy = self.DEFAULT_BUFFER_ENERGY_LIMIT
        if hasattr(self, "spinBox_test_shooter_heat"):
            self.spinBox_test_shooter_heat.setValue(0)

    def _reset_resurrection_state(self, clear_buy_count=False):
        self.resurrection_active = False
        self.resurrection_required_seconds = 0
        self.resurrection_progress = 0
        if clear_buy_count:
            self.buy_revive_count = 0
        self.sentry_weak_active = False
        self.sentry_invincible_remaining_seconds = 0
        self.chassis_power_boost_remaining_seconds = 0
        if hasattr(self, "checkBox_sentry_can_free_revive"):
            self.checkBox_sentry_can_free_revive.setChecked(False)
        if hasattr(self, "checkBox_sentry_can_buy_revive"):
            self.checkBox_sentry_can_buy_revive.setChecked(False)
        if hasattr(self, "checkBox_sentry_is_disengaged"):
            self.checkBox_sentry_is_disengaged.setChecked(False)
        self._update_resurrection_label()

    def _start_resurrection(self):
        remain_seconds = self._get_stage_remain_time()
        self.resurrection_active = True
        self.resurrection_required_seconds = self._calculate_resurrection_required_seconds(remain_seconds)
        self.resurrection_progress = 0
        self.sentry_weak_active = False
        self.sentry_invincible_remaining_seconds = 0
        self.chassis_power_boost_remaining_seconds = 0
        self._reset_power_heat_after_death()
        self.checkBox_sentry_can_free_revive.setChecked(False)
        self.checkBox_sentry_is_disengaged.setChecked(True)
        self._refresh_buy_revive_permission()
        print(
            "[Resurrection] 哨兵战亡，复活读条开始: "
            f"remain={remain_seconds}s required={self.resurrection_required_seconds}s "
            f"buy_revive_count={self.buy_revive_count}"
        )
        self._update_resurrection_label()

    def _sync_resurrection_state(self):
        current_hp = self.spinBox_hp_7.value()
        if self._last_sentry_hp is None:
            self._last_sentry_hp = current_hp

        if current_hp <= 0 and not self.resurrection_active:
            self._start_resurrection()
        elif current_hp > 0 and self.resurrection_active:
            self._reset_resurrection_state(clear_buy_count=False)

        self._last_sentry_hp = current_hp
        if current_hp <= 0 and self.resurrection_active:
            self._refresh_buy_revive_permission()

    def _resurrection_progress_step(self):
        in_supply_buff = self.is_supply_zone_occupied()
        base_low_hp = self.spinBox_hp_base.value() < 2000
        return 4 if in_supply_buff or base_low_hp else 1

    def _tick_resurrection(self):
        if not self.resurrection_active or self.spinBox_hp_7.value() > 0:
            self._tick_post_revive_effects()
            return

        if self.resurrection_required_seconds <= 0:
            return

        if self.resurrection_progress < self.resurrection_required_seconds:
            self.resurrection_progress = min(
                self.resurrection_required_seconds,
                self.resurrection_progress + self._resurrection_progress_step())

        if self.resurrection_progress >= self.resurrection_required_seconds:
            self.checkBox_sentry_can_free_revive.setChecked(True)

        self._refresh_buy_revive_permission()
        self._update_resurrection_label()

    def _tick_post_revive_effects(self):
        if self.sentry_weak_active and self._detect_weak_clear_card():
            self.sentry_weak_active = False
            elapsed_invincible_seconds = 30 - self.sentry_invincible_remaining_seconds
            if elapsed_invincible_seconds >= 10:
                self.sentry_invincible_remaining_seconds = 0
            else:
                self.sentry_invincible_remaining_seconds = 10 - elapsed_invincible_seconds
            print("[Resurrection] 检测到可占领交互模块卡，解除虚弱状态")

        if self.sentry_invincible_remaining_seconds > 0:
            self.sentry_invincible_remaining_seconds -= 1
        if self.chassis_power_boost_remaining_seconds > 0:
            self.chassis_power_boost_remaining_seconds -= 1
        self._update_resurrection_label()

    def _detect_weak_clear_card(self):
        in_supply_buff = self.is_supply_zone_occupied()
        base_buff_detected = self.checkBox_base_buff.isChecked() if hasattr(self, "checkBox_base_buff") else False
        outpost_buff_detected = (
            self.comboBox_outpost_buff.currentIndex() != 0
            if hasattr(self, "comboBox_outpost_buff")
            else False
        )
        return in_supply_buff or base_buff_detected or outpost_buff_detected

    def _refresh_buy_revive_permission(self):
        revive_cost = self.spinBox_sentry_buy_revive_cost.value()
        remaining_gold_coin = self.spinBox_gold_coin.value() if self.spinBox_gold_coin else 0
        can_buy = self.spinBox_hp_7.value() <= 0 and remaining_gold_coin >= revive_cost
        self.checkBox_sentry_can_buy_revive.setChecked(can_buy)

    def _restore_sentry_hp_ratio(self, ratio):
        max_hp = self.spinBox_hp_7.maximum()
        restored_hp = max(1, self._round_half_up(max_hp * ratio))
        self.spinBox_hp_7.setValue(min(max_hp, restored_hp))

    def _restore_sentry_hp_full(self):
        self.spinBox_hp_7.setValue(self.spinBox_hp_7.maximum())

    def _current_chassis_power_limit(self):
        if self.chassis_power_boost_remaining_seconds <= 0:
            return self.DEFAULT_CHASSIS_POWER_LIMIT
        return min(200, self.DEFAULT_CHASSIS_POWER_LIMIT * 2)

    def _update_resurrection_label(self):
        if not hasattr(self, "label_resurrection_status"):
            return

        if self.resurrection_active and self.spinBox_hp_7.value() <= 0:
            self.label_resurrection_status.setText(
                f"{self.resurrection_progress}/{self.resurrection_required_seconds}s "
                f"金币复活{self.buy_revive_count}次"
            )
            return

        effects = []
        if self.sentry_invincible_remaining_seconds > 0:
            effects.append(f"无敌{self.sentry_invincible_remaining_seconds}s")
        if self.sentry_weak_active:
            effects.append("虚弱")
        if self.chassis_power_boost_remaining_seconds > 0:
            effects.append(f"功率提升{self.chassis_power_boost_remaining_seconds}s")
        suffix = " ".join(effects) if effects else "-"
        self.label_resurrection_status.setText(f"{suffix} 金币复活{self.buy_revive_count}次")

    def confirm_free_revive(self):
        if self.spinBox_hp_7.value() > 0:
            return False, "哨兵未战亡，免费复活无效"
        if not self.checkBox_sentry_can_free_revive.isChecked():
            return False, (
                "当前不可免费复活，读条 "
                f"{self.resurrection_progress}/{self.resurrection_required_seconds}s"
            )

        self._restore_sentry_hp_ratio(0.10)
        self.checkBox_sentry_can_free_revive.setChecked(False)
        self.checkBox_sentry_can_buy_revive.setChecked(False)
        self.checkBox_sentry_is_disengaged.setChecked(False)
        self.resurrection_active = False
        self.sentry_invincible_remaining_seconds = 30
        self.sentry_weak_active = True
        self._update_resurrection_label()
        return True, f"免费复活成功，哨兵血量恢复至 {self.spinBox_hp_7.value()}"

    def confirm_buy_revive(self):
        if self.spinBox_hp_7.value() > 0:
            return False, "哨兵未战亡，金币复活无效"
        self._refresh_buy_revive_permission()
        if not self.checkBox_sentry_can_buy_revive.isChecked():
            return False, "当前不可金币复活"

        revive_cost = self.spinBox_sentry_buy_revive_cost.value()
        remaining_gold_coin = self.spinBox_gold_coin.value() if self.spinBox_gold_coin else 0
        if remaining_gold_coin < revive_cost:
            return False, f"剩余金币不足，当前 {remaining_gold_coin}，需要 {revive_cost}"

        if self.spinBox_gold_coin:
            self.spinBox_gold_coin.setValue(max(0, remaining_gold_coin - revive_cost))
        self._restore_sentry_hp_full()
        self.buy_revive_count += 1
        self.checkBox_sentry_can_buy_revive.setChecked(False)
        self.checkBox_sentry_can_free_revive.setChecked(False)
        self.checkBox_sentry_is_disengaged.setChecked(False)
        self.resurrection_active = False
        self.sentry_invincible_remaining_seconds = 3
        self.sentry_weak_active = False
        self.chassis_power_boost_remaining_seconds = 4
        self._update_resurrection_label()
        return True, f"金币复活成功，哨兵血量恢复至 {self.spinBox_hp_7.value()}"

    def confirm_projectile_exchange(self, exchange_target):
        exchange_target = int(exchange_target)
        current_exchange = self.spinBox_sentry_exchange_projectile.value()
        if exchange_target <= current_exchange:
            return True, f"补弹目标未增加，保持累计补弹 {current_exchange}"
        if exchange_target > 300:
            return False, f"补血点买弹上限为 300，收到目标 {exchange_target}"

        delta = exchange_target - current_exchange
        if delta % 10 != 0:
            return False, f"补给区买弹必须以 10 发为单位，收到增量 {delta}"

        remaining_gold_coin = self.spinBox_gold_coin.value() if self.spinBox_gold_coin else 0
        if remaining_gold_coin < delta:
            return False, f"剩余金币不足，当前 {remaining_gold_coin}，需要 {delta}"

        if self.spinBox_gold_coin:
            self.spinBox_gold_coin.setValue(max(0, remaining_gold_coin - delta))
        self.spinBox_ammo.setValue(min(self.spinBox_ammo.maximum(), self.spinBox_ammo.value() + delta))
        self.spinBox_sentry_exchange_projectile.setValue(exchange_target)
        return True, (
            f"补弹成功，累计补弹 {exchange_target}，"
            f"当前 17mm 允许发弹量 {self.spinBox_ammo.value()}"
        )

    def _check_rune_refresh(self, remain_seconds):
        """
        根据比赛剩余时间，自动刷新能量机关状态，增加激活机会次数
        流程：时间到了 -> 设置 sentry_info Bit 14 = 1 (可激活) -> 等待 Service 确认 -> 变为激活中
        """
        
        # 防止同一秒执行多次
        if remain_seconds == self.last_remain_seconds:
            return
        self.last_remain_seconds = remain_seconds

        # === 1. 发放激活机会 (增加计数) ===
        
        # 小符时间点: 7:00(420s), 5:30(330s)
        if remain_seconds in [420, 330]:
            self.small_rune_chances += 1
            print(f"[Rule] 时间 {remain_seconds}s: 获得1次小能量机关激活机会 (当前累积: {self.small_rune_chances})")

        # 大符时间点: 4:00(240s), 2:45(165s), 1:30(90s)
        if remain_seconds in [240, 165, 90]:
            self.big_rune_chances += 1
            print(f"[Rule] 时间 {remain_seconds}s: 获得1次大能量机关激活机会 (当前累积: {self.big_rune_chances})")

        # === 2. 更新 sentry_info 的权限位 (Bit 14) ===
        self._refresh_sentry_activate_flag()

    def confirm_activation(self):
        """
        哨兵发来了确认指令，尝试进入'激活中'状态，消耗累积次数
        供 Plugin 的 Service 回调调用
        """
        # 优先检查 UI 强制勾选 (方便调试)
        ui_checked = False
        if hasattr(self, 'checkBox_can_activate_rune'):
            ui_checked = self.checkBox_can_activate_rune.isChecked()

        # 如果 Flag 为 True (来自自动逻辑) 或 UI 被勾选
        if self.sentry_can_activate_flag or ui_checked:
            
            # === 判定机器人想打哪个符 ===
            # 这里做一个简化的假设：机器人通常先打小符，再打大符。
            # 也可以根据当前比赛时间或机器人发来的姿态进一步判断，但这里简化处理。
            
            activated_type = 0 # 1:小符, 2:大符
            
            current_small_status = self.comboBox_small_rune.currentIndex()
            current_big_status = self.comboBox_big_rune.currentIndex()

            if current_small_status == 1 or current_big_status == 1:
                active_rune_name = "small" if current_small_status == 1 else "big"
                print(
                    f">>> [Ignore] {self._get_rune_label(active_rune_name)}已处于正在激活窗口内，"
                    "忽略重复激活请求"
                )
                return True

            remain_seconds = self._get_stage_remain_time()
            prefer_big_rune = remain_seconds <= 240

            # 大符时间窗内优先消耗大符次数，避免 90s / 165s / 240s 仍然误打小符
            if prefer_big_rune and self.big_rune_chances > 0 and current_big_status == 0:
                activated_type = 2
                self.big_rune_chances -= 1
                print(f">>> [Success] 消耗1次大符机会 (剩余: {self.big_rune_chances})")

            # 逻辑：如果小符有次数且没激活 -> 激活小符
            elif self.small_rune_chances > 0 and current_small_status == 0:
                activated_type = 1
                self.small_rune_chances -= 1 # 扣除次数
                print(f">>> [Success] 消耗1次小符机会 (剩余: {self.small_rune_chances})")
                
            # 否则：如果大符有次数且没激活 -> 激活大符
            elif self.big_rune_chances > 0 and current_big_status == 0:
                activated_type = 2
                self.big_rune_chances -= 1 # 扣除次数
                print(f">>> [Success] 消耗1次大符机会 (剩余: {self.big_rune_chances})")
                
            # 特殊情况：如果是 UI 强制勾选但次数为0 (调试模式)
            elif ui_checked:
                 if current_small_status == 0: activated_type = 1
                 elif current_big_status == 0: activated_type = 2
                 print(">>> [Debug] UI强制激活")

            # === 执行激活 ===
            if activated_type > 0:
                if activated_type == 1:
                    self._set_rune_status("small", 1)
                elif activated_type == 2:
                    self._set_rune_status("big", 1)

                self._refresh_sentry_activate_flag()
                return True
            else:
                print(">>> [Fail] 虽然有权限，但没有可激活的目标 (可能都已激活)")
                return False
        else:
            print(">>> [Fail] 激活拒绝：无累积次数")
            return False
        
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MatchControlWidget()
    window.setWindowTitle("RM Match Server Sim")
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
