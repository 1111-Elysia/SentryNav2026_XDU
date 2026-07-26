// adaptive_mppi_panel.cpp — RViz 仪表盘面板实现

#include "pri_adaptive_mppi/adaptive_mppi_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <cmath>
#include <sstream>
#include <pluginlib/class_list_macros.hpp>

namespace pri_adaptive_mppi
{

AdaptiveMppiPanel::AdaptiveMppiPanel(QWidget * parent)
  : rviz_common::Panel(parent)
{
  auto * main_layout = new QVBoxLayout;

  // ── 模式显示 ──
  mode_label_ = new QLabel("NORMAL");
  mode_label_->setAlignment(Qt::AlignCenter);
  mode_label_->setStyleSheet(
    "QLabel { font-size: 28px; font-weight: bold; color: white; "
    "background-color: #2ecc71; border-radius: 6px; padding: 8px; }");
  main_layout->addWidget(mode_label_);

  // ── 速度数值 ──
  auto * vel_layout = new QHBoxLayout;

  vx_label_ = new QLabel("vx: ---");
  vx_label_->setStyleSheet("QLabel { font-size: 16px; color: #ddd; }");
  vel_layout->addWidget(vx_label_);

  vy_label_ = new QLabel("vy: ---");
  vy_label_->setStyleSheet("QLabel { font-size: 16px; color: #ddd; }");
  vel_layout->addWidget(vy_label_);

  main_layout->addLayout(vel_layout);

  // ── 速度条 ──
  speed_bar_ = new QProgressBar;
  speed_bar_->setRange(0, 100);
  speed_bar_->setValue(0);
  speed_bar_->setTextVisible(true);
  speed_bar_->setFormat("Speed: 0.00 m/s");
  speed_bar_->setStyleSheet(
    "QProgressBar { border: 1px solid #555; border-radius: 4px; "
    "text-align: center; height: 24px; }"
    "QProgressBar::chunk { background-color: #3498db; border-radius: 3px; }");
  main_layout->addWidget(speed_bar_);

  setLayout(main_layout);
}

AdaptiveMppiPanel::~AdaptiveMppiPanel() = default;

void AdaptiveMppiPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/FollowPath/dashboard", rclcpp::QoS(10),
    std::bind(&AdaptiveMppiPanel::dashboardCallback, this, std::placeholders::_1));
}

void AdaptiveMppiPanel::dashboardCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // 解析 "MODE,vx,vy"
  std::istringstream ss(msg->data);
  std::string mode_str;
  double vx = 0, vy = 0;
  char comma;
  std::getline(ss, mode_str, ',');
  ss >> vx >> comma >> vy;

  double speed = std::hypot(vx, vy);

  // 更新模式标签颜色
  QString bg, text_color = "white";
  if (mode_str == "UPHILL") {
    bg = "#e67e22";  // 橙色
  } else if (mode_str == "DOWNHILL") {
    bg = "#e74c3c";  // 红色
  } else {
    bg = "#2ecc71";  // 绿色
  }

  QString mode_style = QString(
    "QLabel { font-size: 28px; font-weight: bold; color: %1; "
    "background-color: %2; border-radius: 6px; padding: 8px; }")
    .arg(text_color, bg);

  // 必须在 Qt 主线程更新 UI
  QMetaObject::invokeMethod(this, [this, mode_str, vx, vy, speed, mode_style]() {
    mode_label_->setText(QString::fromStdString(mode_str));
    mode_label_->setStyleSheet(mode_style);

    vx_label_->setText(QString("vx: %1").arg(vx, 0, 'f', 2));
    vy_label_->setText(QString("vy: %1").arg(vy, 0, 'f', 2));

    int pct = static_cast<int>(std::min(speed * 50.0, 100.0));  // 2.0 m/s = 100%
    speed_bar_->setValue(pct);
    speed_bar_->setFormat(QString("Speed: %1 m/s").arg(speed, 0, 'f', 2));
  }, Qt::QueuedConnection);
}

}  // namespace pri_adaptive_mppi

PLUGINLIB_EXPORT_CLASS(pri_adaptive_mppi::AdaptiveMppiPanel, rviz_common::Panel)
