#!/usr/bin/env python3

import sys
from os import path

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
from rm_referee_mock.widget_confirmed_line_edit import ConfirmedLineEdit

from ament_index_python.packages import get_package_share_directory

PACKAGE_SHARE = get_package_share_directory("rm_referee_mock")


class MatchControlWidget(QtWidgets.QWidget):
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

        self.init_ui()

        self.current_hurt_armor_id = 0
        self.trigger_hurt = False
        self.sentry_can_activate_flag = False
        self.authorized_rune_type = 0
        self.rune_activating_timer = 0
        self.small_rune_chances = 0
        self.big_rune_chances = 0
        self.last_remain_seconds = -1

    def init_ui(self):
        """初始化 UI 控件的默认值和选项"""
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
        dart_targets = ["未击中", "前哨站", "基地"]
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

        self.label_stage_countdown = QtWidgets.QLabel(self)
        self.label_stage_countdown.setObjectName("label_stage_countdown")
        self.label_stage_countdown.setGeometry(430, 515, 210, 20)
        self.label_stage_countdown.setStyleSheet("color: #666666;")
        self._on_game_stage_changed(self.comboBox_game_stage.currentIndex())

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
        self.groupBox_robot_status.setGeometry(650, 20, 120, 90)
        projectile_group.setGeometry(650, 120, 120, 210)
        if hasattr(self, "groupBox_hurt"):
            self.groupBox_hurt.setGeometry(650, 340, 120, 120)
        if hasattr(self, "groupBox_service_echo"):
            self.groupBox_service_echo.setGeometry(650, 470, 120, 120)

    def _is_rmul_game_type(self):
        return self.comboBox_game_type.currentIndex() in self.RMUL_GAME_TYPES

    def _on_game_stage_changed(self, stage_index):
        if stage_index in self.PRE_MATCH_STAGE_DURATIONS:
            self.stage_countdown_remaining = self.PRE_MATCH_STAGE_DURATIONS[stage_index]
            self._update_stage_countdown_display()
            self._set_countdown_running(True)
            return

        self.stage_countdown_remaining = 0
        self._update_stage_countdown_display()
        if stage_index in (0, 1, self.FINISHED_STAGE_INDEX):
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
        if self.comboBox_game_stage.currentIndex() in self.PRE_MATCH_STAGE_DURATIONS:
            self._tick_stage_countdown()
            return

        self._tick_match_countdown()

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
            按 2026 V1.2.0 协议计算 EventData 位图值。
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

        event_val |= int(self.checkBox_2.isChecked()) << 0
        event_val |= int(self.checkBox.isChecked()) << 1
        if self._is_rmul_game_type():
            event_val |= int(self.checkBox_3.isChecked()) << 2

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

    def get_game_status(self):
        """获取当前比赛状态"""
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
                "outpost": self.spinBox_hp_outpost.value(),
                "base": self.spinBox_hp_base.value(),
            },
            # 详细机器人状态
            "robot_status": {
                "id": self.spinBox_robot_id.value(),
                "current_hp": self.spinBox_hp_7.value(), # 复用哨兵血量
                "max_hp": 600,
                "ammo": self.spinBox_ammo.value()
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
            # 伤害数据
            "hurt_data": {
                "armor_id": self.current_hurt_armor_id if self.trigger_hurt else 0,
                # 如果 trigger 为 false, 发 0 或者不发(plugin处理)
                "trigger": self.trigger_hurt 
            },
            # === 新增：传出可激活标志，用于 sentry_info Bit 14 ===
            "sentry_can_activate": self.sentry_can_activate_flag
        }
        if self.trigger_hurt:
            self.trigger_hurt = False

        return status

    def update_sentry_echo(self, mode):
        """更新 UI 显示收到的指令"""
        if hasattr(self, 'label_sentry_mode'):
            # 简单映射模式名
            names = {0: "无效", 1: "进攻", 2: "防御", 3: "移动"}
            txt = names.get(mode, str(mode))
            self.label_sentry_mode.setText(f"Mode: {txt}")
            # 可以变个色提示更新
            self.label_sentry_mode.setStyleSheet("color: red; font-weight: bold;")
            QTimer.singleShot(500, lambda: self.label_sentry_mode.setStyleSheet("color: blue; font-weight: bold;"))    

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

        current_small_status = self.comboBox_small_rune.currentIndex() # 0:未激活
        current_big_status = self.comboBox_big_rune.currentIndex()     # 0:未激活
        
        can_activate = False
        
        # 有小符机会 且 小符未激活
        if self.small_rune_chances > 0 and current_small_status == 0:
            can_activate = True
            
        # 有大符机会 且 大符未激活
        if self.big_rune_chances > 0 and current_big_status == 0:
            can_activate = True
            
        # 同步给 flag，用于发布给机器人
        self.sentry_can_activate_flag = can_activate
        
        # 同步 UI 勾选框 (视觉反馈)
        if hasattr(self, 'checkBox_can_activate_rune'):
            # 如果人工没介入，自动更新
            if self.sentry_can_activate_flag:
                self.checkBox_can_activate_rune.setChecked(True)

        # === 3. 倒计时逻辑  ===
        # 如果处于"正在激活"(Index=1)，处理20s倒计时
        if current_small_status == 1 or current_big_status == 1:
            if self.rune_activating_timer > 0:
                self.rune_activating_timer -= 1
                if self.rune_activating_timer == 0:
                    print("[Timeout] 激活时间窗口结束，重置状态")
                    if current_small_status == 1: self.comboBox_small_rune.setCurrentIndex(0)
                    if current_big_status == 1: self.comboBox_big_rune.setCurrentIndex(0)


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

            # 逻辑：如果小符有次数且没激活 -> 激活小符
            if self.small_rune_chances > 0 and current_small_status == 0:
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
                self.rune_activating_timer = 20 # 开启20秒窗口
                
                if activated_type == 1:
                    self.comboBox_small_rune.setCurrentIndex(1) # 设为: 正在激活
                elif activated_type == 2:
                    self.comboBox_big_rune.setCurrentIndex(1) # 设为: 正在激活
                
                # 激活成功后，如果手里没有剩余次数了，就把勾选框取消
                # 如果还有次数（比如累积了2次），就保持勾选，允许机器人接着打
                total_left = self.small_rune_chances + self.big_rune_chances
                if total_left == 0:
                    self.sentry_can_activate_flag = False
                    if hasattr(self, 'checkBox_can_activate_rune'):
                        self.checkBox_can_activate_rune.setChecked(False)
                
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
