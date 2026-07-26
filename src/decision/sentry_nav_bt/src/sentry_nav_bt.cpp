// STD
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <filesystem>
#include <algorithm>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// 行为树节点
#include "sentry_nav_bt/check_condition.hpp"
#include "sentry_nav_bt/goal_selector_node.hpp"
#include "sentry_nav_bt/random_selector_node.hpp"
#include "sentry_nav_bt/topic_listener.hpp"
#include "sentry_nav_bt/print_blackboard_node.hpp"
#include "sentry_nav_bt/compare_values.hpp"
#include "sentry_nav_bt/print_node.hpp"
#include "sentry_nav_bt/set_blackboard.hpp"
#include "sentry_nav_bt/referee_actions.hpp"
#include "sentry_nav_bt/patrol_nodes.hpp"
#include "sentry_nav_bt/reliable_navigate_to_pose.hpp"
#include "sentry_nav_bt/publish_vw_action.hpp"
#include "sentry_nav_bt/runtime_config.hpp"

namespace
{

class ValidationWaitAction : public BT::SyncActionNode
{
public:
    ValidationWaitAction(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("wait_duration", 0.0, "Validation-only wait duration")};
    }

    BT::NodeStatus tick() override
    {
        return BT::NodeStatus::SUCCESS;
    }
};

class WaitAction : public BT::StatefulActionNode
{
public:
    WaitAction(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("wait_duration", 0.0, "Wait duration in seconds")};
    }

    BT::NodeStatus onStart() override
    {
        double wait_duration = 0.0;
        getInput("wait_duration", wait_duration);
        if (wait_duration <= 0.0) {
            return BT::NodeStatus::SUCCESS;
        }

        deadline_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(wait_duration));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        return std::chrono::steady_clock::now() >= deadline_
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    std::chrono::steady_clock::time_point deadline_;
};

bool registerBehaviorTreesFromDirectory(
    BT::BehaviorTreeFactory &factory,
    const std::string &directory,
    const rclcpp::Logger &logger)
{
    namespace fs = std::filesystem;

    if (directory.empty()) {
        return true;
    }

    std::error_code ec;
    const fs::path dir_path(directory);
    if (!fs::exists(dir_path, ec) || !fs::is_directory(dir_path, ec)) {
        RCLCPP_ERROR(logger, "行为树子树目录不存在或不是目录: %s", directory.c_str());
        return false;
    }

    std::vector<fs::path> xml_files;
    for (const auto &entry : fs::recursive_directory_iterator(dir_path, ec)) {
        if (ec) {
            RCLCPP_ERROR(logger, "遍历行为树子树目录失败: %s (%s)", directory.c_str(), ec.message().c_str());
            return false;
        }
        if (entry.is_regular_file() && entry.path().extension() == ".xml") {
            xml_files.push_back(entry.path());
        }
    }

    std::sort(xml_files.begin(), xml_files.end());
    for (const auto &xml_file : xml_files) {
        try {
            factory.registerBehaviorTreeFromFile(xml_file.string());
            RCLCPP_INFO(logger, "已注册行为树子树: %s", xml_file.string().c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(
                logger,
                "注册行为树子树失败: %s (%s)",
                xml_file.string().c_str(),
                e.what());
            return false;
        }
    }

    if (xml_files.empty()) {
        RCLCPP_WARN(logger, "行为树子树目录为空: %s", directory.c_str());
    }

    return true;
}

void useValidationOnlyNodes(BT::BehaviorTreeFactory &factory)
{
    factory.unregisterBuilder("Wait");
    factory.registerNodeType<ValidationWaitAction>("Wait");
}

BT::Tree createRegisteredBehaviorTree(
    BT::BehaviorTreeFactory &factory,
    const std::string &main_tree_file,
    const std::string &main_tree_id,
    const BT::Blackboard::Ptr &blackboard)
{
    factory.registerBehaviorTreeFromFile(main_tree_file);
    return factory.createTree(main_tree_id, blackboard);
}

}  // namespace

void RegisterBehaviorTreePlugins(BT::BehaviorTreeFactory &factory,
                                 const rclcpp::Node::SharedPtr &node)
{
    BT::NodeBuilder reliable_navigate_builder =
    [node](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<sentry_nav_bt::ReliableNavigateToPose>(name, config, node);
    };
    factory.registerBuilder<sentry_nav_bt::ReliableNavigateToPose>(
        "ReliableNavigateToPose", reliable_navigate_builder);
    factory.registerNodeType<WaitAction>("Wait");
    // 打印黑板值
    BT::NodeBuilder print_blackboard_builder =
        [](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<sentry_nav_bt::PrintBlackboardValue>(name, config);
    };

    // 注册裁判系统交互节点
    factory.registerNodeType<sentry_nav_bt::MaintainSentryPosture>("MaintainSentryPosture");
    factory.registerNodeType<sentry_nav_bt::ResolveSentryPosture>("ResolveSentryPosture");
    factory.registerNodeType<sentry_nav_bt::ConfirmResurrection>("ConfirmResurrection");
    factory.registerNodeType<sentry_nav_bt::BuySentryProjectile>("BuySentryProjectile");
    factory.registerNodeType<sentry_nav_bt::EngageRune>("EngageRune");
    factory.registerNodeType<sentry_nav_bt::EngageOutpost>("EngageOutpost");
    factory.registerNodeType<sentry_nav_bt::PublishScanMode>("PublishScanMode");
    factory.registerNodeType<sentry_nav_bt::PublishVw>("PublishVw");
    factory.registerBuilder<sentry_nav_bt::PrintBlackboardValue>("PrintBlackboardValue", print_blackboard_builder);
    // 条件检查
    factory.registerNodeType<sentry_nav_bt::CheckCondition>("CheckCondition");
    // 关键值比较
    factory.registerNodeType<sentry_nav_bt::CompareValues>("CompareValues");
    // <CompareValues first_key="distance" second_key="threshold" comparison="lt"/>
    // 随机选择器
    factory.registerNodeType<sentry_nav_bt::RandomSelector>("RandomSelector");
    // 目标选择器
    factory.registerNodeType<sentry_nav_bt::GoalSelector>("GoalSelector");
    factory.registerNodeType<sentry_nav_bt::PatrolGoalSelector>("PatrolGoalSelector");
    factory.registerNodeType<sentry_nav_bt::CheckGoalReached>("CheckGoalReached");
    factory.registerNodeType<sentry_nav_bt::UseTrackingPlanner>("UseTrackingPlanner");
    // 打印节点
    factory.registerNodeType<sentry_nav_bt::PrintNode>("PrintNode");
    // 设置黑板值
    factory.registerNodeType<sentry_nav_bt::SetBlackboardValue>("SetBlackboardValue");

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sentry_nav_bt");

    // 从参数获取行为树XML文件名
    std::string bt_xml_filename;
    std::string bt_main_tree_id;
    node->declare_parameter("bt_xml_filename", "");
    node->declare_parameter("bt_main_tree_id", "MainTree");
    if (!node->get_parameter("bt_xml_filename", bt_xml_filename))
    {
        RCLCPP_ERROR(node->get_logger(), "无法获取参数 'bt_xml_filename'");
        return 1;
    }
    node->get_parameter("bt_main_tree_id", bt_main_tree_id);

    // 获取路径点文件参数
    std::string waypoints_file;
    std::string bt_subtree_dir;
    bool validate_bt_only = false;

    node->declare_parameter("waypoints_file", "");
    node->declare_parameter("bt_subtree_dir", "");
    node->declare_parameter("validate_bt_only", false);

    node->get_parameter("waypoints_file", waypoints_file);
    node->get_parameter("bt_subtree_dir", bt_subtree_dir);
    node->get_parameter("validate_bt_only", validate_bt_only);

    // 创建行为树工厂
    BT::BehaviorTreeFactory factory;
    RegisterBehaviorTreePlugins(factory,node);
    if (validate_bt_only) {
        useValidationOnlyNodes(factory);
    }
    if (!registerBehaviorTreesFromDirectory(factory, bt_subtree_dir, node->get_logger())) {
        return 1;
    }

    // 打印可用行为树节点
    RCLCPP_INFO(node->get_logger(), "可用的行为树节点:");

    // 从XML创建行为树
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    // 创建黑板管理器
    auto bb_manager = std::make_shared<sentry_nav_bt::BlackboardManager>(node, blackboard);

    bb_manager->bb_manager_init();

    if (waypoints_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "路径点文件参数 'waypoints_file' 未设置！");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "加载路径点: %s", waypoints_file.c_str());
    bb_manager->load_waypoints(waypoints_file);

    auto runtime_config = std::make_shared<sentry_nav_bt::RuntimeConfigManager>(
        node, blackboard);
    if (!runtime_config->applyCurrentParameters()) {
        return 1;
    }

    if (validate_bt_only)
    {
        try
        {
            auto tree = createRegisteredBehaviorTree(
                factory,
                bt_xml_filename,
                bt_main_tree_id,
                blackboard);
            RCLCPP_INFO(
                node->get_logger(),
                "行为树 XML 校验通过: %s (main_tree_id=%s)",
                bt_xml_filename.c_str(),
                bt_main_tree_id.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "行为树 XML 校验失败: %s", e.what());
            return 1;
        }

        rclcpp::shutdown();
        return 0;
    }

    // std::this_thread::sleep_for(std::chrono::seconds(5));

    // 等待导航action server启动
    RCLCPP_INFO(node->get_logger(), "等待导航服务器启动...");

    // 创建一个临时的action client来检查服务器是否可用
    auto navigate_action_client =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node, "navigate_to_pose");

    // 等待服务器成为可用状态，最多等待60秒
    bool server_available = false;
    auto start_time = node->now();
    while (rclcpp::ok() && !server_available)
    {
        rclcpp::spin_some(node);
        server_available = navigate_action_client->wait_for_action_server(std::chrono::seconds(1));
        if (!server_available)
        {
            if ((node->now() - start_time).seconds() > 60.0)
            {
                RCLCPP_ERROR(node->get_logger(), "导航服务器在60秒内未启动，退出程序");
                return 1;
            }
            RCLCPP_INFO(node->get_logger(), "等待导航服务器...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_INFO(node->get_logger(), "导航服务器已可用");

    // 加载路径点后，发布初始位姿
    auto initial_pose_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);

    // 从黑板获取初始位姿
    geometry_msgs::msg::PoseStamped init_pose;
    std::string init_pose_key = "waypoint_init"; // 首先尝试指定的初始位姿键

    bool pose_found = false;

    // 尝试从黑板读取指定的初始位姿
    if (blackboard->get(init_pose_key, init_pose))
    {
        pose_found = true;
        RCLCPP_INFO(node->get_logger(), "找到指定的初始位姿 '%s'", init_pose_key.c_str());
    }
    // 如果都没有，尝试使用起点
    else if (blackboard->get("waypoint_start", init_pose))
    {
        pose_found = true;
        RCLCPP_INFO(node->get_logger(), "使用起点作为初始位姿");
    }

    if (pose_found)
    {
        // 转换为 PoseWithCovarianceStamped 并设置协方差
        geometry_msgs::msg::PoseWithCovarianceStamped init_pose_cov;

        // 复制 Header
        init_pose_cov.header = init_pose.header;
        init_pose_cov.header.stamp = node->now(); // 刷新时间戳

        // 确保帧ID正确
        if (init_pose_cov.header.frame_id.empty())
        {
            init_pose_cov.header.frame_id = "map";
        }

        // 复制 Pose
        init_pose_cov.pose.pose = init_pose.pose;

        // 设置协方差 (AMCL/Nav2 需要这个来确认定位可信度)
        // 0.25 是一个常见的经验值
        for(int i=0; i<36; i++) { init_pose_cov.pose.covariance[i] = 0.0; }
        init_pose_cov.pose.covariance[0] = 0.25;  // X 轴方差
        init_pose_cov.pose.covariance[7] = 0.25;  // Y 轴方差
        init_pose_cov.pose.covariance[35] = 0.0685; // Yaw 轴方差 (约等于 15度)

        // 发布初始位姿
        RCLCPP_INFO(node->get_logger(),
                    "发布初始位姿 (标准格式): [%.2f, %.2f, %.2f]",
                    init_pose_cov.pose.pose.position.x,
                    init_pose_cov.pose.pose.position.y,
                    init_pose_cov.pose.pose.position.z);

        // 多次发布以确保接收
        for (int i = 0; i < 3; i++)
        {
            initial_pose_pub->publish(init_pose_cov);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "未能找到有效的初始位姿信息");
    }

    try
    {
        auto tree = createRegisteredBehaviorTree(
            factory,
            bt_xml_filename,
            bt_main_tree_id,
            blackboard);

        auto groot_publisher = std::make_shared<BT::Groot2Publisher>(tree, 1667);
        RCLCPP_INFO(node->get_logger(), "Groot2 Publisher started. Port: 1667");
        
        RCLCPP_INFO(node->get_logger(), "运行导航行为树");
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);
            tree.tickOnce();
            tree.sleep(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "异常: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
