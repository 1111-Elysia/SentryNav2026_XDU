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

    // 这里只创建 client，真正的 node SharedPtr 在 main 里塞进 ctx.node
    ctx.client = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
      // 发布 /initialpose
    ctx.initialpose_pub = this->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10);

    RCLCPP_INFO(get_logger(),
                "等待 Nav2 navigate_to_pose Action 服务器...");
    ctx.client->wait_for_action_server();
    RCLCPP_INFO(get_logger(),
                "Nav2 Action 服务器已就绪");

    // 行为树工厂 & 注册节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<InitPoints>("InitPoints");
    factory.registerNodeType<NextPoint>("NextPoint");
    factory.registerNodeType<SendNav2Goal>("SendNav2Goal");
    factory.registerNodeType<PublishInitialPose>("PublishInitialPose");
    factory.registerNodeType<ColorToGoal>("ColorToGoal");
    factory.registerNodeType<WaitSeconds>("WaitSeconds");

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

    timer_ = create_wall_timer(100ms, [this]()
                               {
      auto status = tree_.tickRoot();
      if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(get_logger(), "Behavior tree root FAILURE");
      } });
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
