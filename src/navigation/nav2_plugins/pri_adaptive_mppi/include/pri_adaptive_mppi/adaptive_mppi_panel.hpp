// adaptive_mppi_panel.hpp — RViz 仪表盘面板

#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>

namespace pri_adaptive_mppi
{

class AdaptiveMppiPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AdaptiveMppiPanel(QWidget * parent = nullptr);
  ~AdaptiveMppiPanel() override;

  void onInitialize() override;

private:
  void dashboardCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  QLabel * mode_label_;
  QLabel * vx_label_;
  QLabel * vy_label_;
  QProgressBar * speed_bar_;
};

}  // namespace pri_adaptive_mppi
