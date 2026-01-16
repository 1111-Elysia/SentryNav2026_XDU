// pub_point_bt/src/bt_runner.cpp

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// 直接包含节点实现
#include "bt_nodes.cpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class PubPointBTNode : public rclcpp::Node
{
public:
  PubPointBTNode()
      : Node("pub_point_bt_node")
  {
    auto &ctx = BtRosContext::instance();

    // 1. 初始化 Nav2 Client
    ctx.client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    ctx.initialpose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    if (!ctx.client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "Nav2 Action Server not available after waiting");
    }
    RCLCPP_INFO(get_logger(), "Nav2 Action Server is ready.");
    
    // 2. 初始化 TF Listener (用于 CheckDistance 读取坐标)
    ctx.tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    ctx.tf_listener = std::make_shared<tf2_ros::TransformListener>(*ctx.tf_buffer);

    // 3. 注册节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<PublishInitialPose>("PublishInitialPose");
    factory.registerNodeType<InitPoints>("InitPoints");
    factory.registerNodeType<NextPoint>("NextPoint");
    factory.registerNodeType<SendNav2Goal>("SendNav2Goal");
    factory.registerNodeType<CheckDistance>("CheckDistance");
    
    // XML 路径（默认安装在 share/pub_point_bt/config/waypoints_bt.xml）
    std::string xml_path = this->declare_parameter<std::string>(
        "bt_xml",
        ament_index_cpp::get_package_share_directory("pub_point_bt") +
            std::string("/config/waypoints_bt.xml"));

    RCLCPP_INFO(get_logger(), "Loading BT XML: %s", xml_path.c_str());

    tree_ = factory.createTreeFromFile(xml_path);

    try
    {
      publisher_zmq_ = std::make_shared<BT::PublisherZMQ>(tree_, 1919, 1920);
      RCLCPP_INFO(get_logger(),
                  "BT ZMQ publisher started on ports 1919 (pub) / 1920 (server)");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start BT ZMQ publisher: %s", e.what());
    }

    timer_ = create_wall_timer(100ms, [this]() {
      auto status = tree_.tickRoot();

      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(get_logger(), "🛑 任务结束 (状态: %s)，停止运行！", BT::toStr(status).c_str());
        this->timer_->cancel(); // 关键：取消定时器，不再触发
        // rclcpp::shutdown();  // 如果想连节点一起关掉，可以把这行解注
      }
    });
  }

private:
  BT::Tree tree_;
  std::shared_ptr<BT::PublisherZMQ> publisher_zmq_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PubPointBTNode>();

  // 这里把 SharedPtr 塞进 BtRosContext，供所有 BT 节点使用
  BtRosContext::instance().node = node;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
