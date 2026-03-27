// STD
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <ctime>
#include <filesystem>
#include <fstream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Nav2 plugins
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"
#include "nav2_behavior_tree/plugins/action/wait_action.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"
#include "nav2_behavior_tree/plugins/decorator/rate_controller.hpp"

// 裁判系统
#include "rm_referee_msgs/msg/robot_status.hpp"

// 行为树节点
#include "sentry_nav_bt_test/check_condition.hpp"
#include "sentry_nav_bt_test/goal_selector_node.hpp"
#include "sentry_nav_bt_test/random_selector_node.hpp"
#include "sentry_nav_bt_test/topic_listener.hpp"
#include "sentry_nav_bt_test/print_blackboard_node.hpp"
#include "sentry_nav_bt_test/compare_values.hpp"
#include "sentry_nav_bt_test/print_node.hpp"
#include "sentry_nav_bt_test/set_blackboard.hpp"
#include "sentry_nav_bt_test/referee_actions.hpp"
#include "sentry_nav_bt_test/auto_aim_and_fire_action.hpp"
#include "sentry_nav_bt_test/chase_target_action.hpp"
#include "sentry_nav_bt_test/patrol_nodes.hpp"
#include "sentry_nav_bt_test/reliable_navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

namespace
{

std::string getCurrentDateString()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);

    char buffer[16];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d", &local_tm);
    return std::string(buffer);
}

std::string resolveBtMessageLogFilePath(const std::string &configured_path)
{
    namespace fs = std::filesystem;

    fs::path configured(configured_path);
    fs::path directory_path = configured.parent_path();
    std::string file_stem = configured.stem().string();
    std::string extension = configured.extension().string();

    if (!configured.has_extension()) {
        directory_path = configured;
        file_stem = "sentry_nav_bt_messages";
        extension = ".log";
    }

    if (directory_path.empty()) {
        directory_path = ".";
    }
    if (file_stem.empty()) {
        file_stem = "sentry_nav_bt_messages";
    }
    if (extension.empty()) {
        extension = ".log";
    }

    const fs::path dated_log_path =
        directory_path / (file_stem + "_" + getCurrentDateString() + extension);
    return dated_log_path.string();
}

bool initializeBtMessageLogFile(const std::string &log_file_path, const rclcpp::Logger &logger)
{
    namespace fs = std::filesystem;

    const fs::path log_path(log_file_path);
    std::error_code ec;
    const fs::path directory_path = log_path.parent_path();
    if (!directory_path.empty() && !fs::exists(directory_path, ec)) {
        fs::create_directories(directory_path, ec);
        if (ec) {
            RCLCPP_WARN(
                logger,
                "无法创建行为树消息日志目录: %s (%s)",
                directory_path.string().c_str(),
                ec.message().c_str());
            return false;
        }
    }

    const bool needs_header =
        !fs::exists(log_path, ec) || (ec ? false : fs::file_size(log_path, ec) == 0U);

    std::ofstream ofs(log_file_path, std::ios::app);
    if (!ofs.is_open()) {
        RCLCPP_WARN(logger, "无法初始化行为树消息日志文件: %s", log_file_path.c_str());
        return false;
    }

    if (needs_header) {
        ofs << "# sentry_nav_bt_test behavior tree messages\n";
    }

    return true;
}

}  // namespace

void RegisterBehaviorTreePlugins(BT::BehaviorTreeFactory &factory,
                                 const rclcpp::Node::SharedPtr &node)
{
    // 注册节点
    // NavigateToPose
    BT::NodeBuilder navigate_builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
            name, "navigate_to_pose", config);
    };
    factory.registerBuilder<nav2_behavior_tree::NavigateToPoseAction>("NavigateToPose", navigate_builder);
    BT::NodeBuilder reliable_navigate_builder =
    [node](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<sentry_nav_bt_test::ReliableNavigateToPose>(name, config, node);
    };
    factory.registerBuilder<sentry_nav_bt_test::ReliableNavigateToPose>(
        "ReliableNavigateToPose", reliable_navigate_builder);
    // 等待
    BT::NodeBuilder wait_builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<nav2_behavior_tree::WaitAction>(
            name, "wait", config);
    };
    factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", wait_builder);
    // 打印黑板值
    BT::NodeBuilder print_blackboard_builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<sentry_nav_bt_test::PrintBlackboardValue>(name, config);
    };

    // 注册裁判系统交互节点
    factory.registerNodeType<sentry_nav_bt_test::SetSentryPosture>("SetSentryPosture");
    factory.registerNodeType<sentry_nav_bt_test::RequestActivateRune>("RequestActivateRune");
    factory.registerNodeType<sentry_nav_bt_test::ConfirmResurrection>("ConfirmResurrection");
    // 注册自瞄节点
    factory.registerNodeType<sentry_nav_bt_test::AutoAimAndFire>("AutoAimAndFire");
    factory.registerBuilder<sentry_nav_bt_test::PrintBlackboardValue>("PrintBlackboardValue", print_blackboard_builder);
    // 条件检查
    factory.registerNodeType<sentry_nav_bt_test::CheckCondition>("CheckCondition");
    // 关键值比较
    factory.registerNodeType<sentry_nav_bt_test::CompareValues>("CompareValues");
    // <CompareValues first_key="distance" second_key="threshold" comparison="lt"/>
    // 随机选择器
    factory.registerNodeType<sentry_nav_bt_test::RandomSelector>("RandomSelector");
    // 目标选择器
    factory.registerNodeType<sentry_nav_bt_test::GoalSelector>("GoalSelector");
    factory.registerNodeType<sentry_nav_bt_test::PatrolGoalSelector>("PatrolGoalSelector");
    factory.registerNodeType<sentry_nav_bt_test::CheckGoalReached>("CheckGoalReached");
    // 打印节点
    factory.registerNodeType<sentry_nav_bt_test::PrintNode>("PrintNode");
    // 设置黑板值
    factory.registerNodeType<sentry_nav_bt_test::SetBlackboardValue>("SetBlackboardValue");

    BT::NodeBuilder chase_builder =
    [node](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<sentry_nav_bt_test::ChaseTargetAction>(name, config, node);
    };
    factory.registerBuilder<sentry_nav_bt_test::ChaseTargetAction>("ChaseTarget", chase_builder);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sentry_nav_bt_test");

    // 从参数获取行为树XML文件名
    std::string bt_xml_filename;
    node->declare_parameter("bt_xml_filename", "");
    if (!node->get_parameter("bt_xml_filename", bt_xml_filename))
    {
        RCLCPP_ERROR(node->get_logger(), "无法获取参数 'bt_xml_filename'");
        return 1;
    }

    // 获取路径点文件参数
    std::string waypoints_red_file;
    std::string waypoints_blue_file;
    std::string bt_message_log_file;

    node->declare_parameter("waypoints_red_file", "");
    node->declare_parameter("waypoints_blue_file", "");
    node->declare_parameter("bt_message_log_file", "/tmp/sentry_nav_bt_messages.log");

    node->get_parameter("waypoints_red_file", waypoints_red_file);
    node->get_parameter("waypoints_blue_file", waypoints_blue_file);
    node->get_parameter("bt_message_log_file", bt_message_log_file);

    // 创建行为树工厂
    BT::BehaviorTreeFactory factory;
    RegisterBehaviorTreePlugins(factory,node);

    // 打印可用行为树节点
    RCLCPP_INFO(node->get_logger(), "可用的行为树节点:");

    // 从XML创建行为树
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    if (!bt_message_log_file.empty())
    {
        bt_message_log_file = resolveBtMessageLogFilePath(bt_message_log_file);
        if (initializeBtMessageLogFile(bt_message_log_file, node->get_logger())) {
            RCLCPP_INFO(node->get_logger(), "行为树消息日志文件: %s", bt_message_log_file.c_str());
        }
    }

    blackboard->set("bt_message_log_file", bt_message_log_file);

    // 创建黑板管理器
    auto bb_manager = std::make_shared<sentry_nav_bt_test::BlackboardManager>(node, blackboard);

    bb_manager->bb_manager_init();

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

    // ======== 添加临时机器人ID监听器 ========
    RCLCPP_INFO(node->get_logger(), "等待获取机器人ID...");

    // 用于同步的标志
    std::atomic<bool> id_received(false);
    uint8_t robot_id = 0; // 默认ID

    // 配置QoS
    auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    qos.best_effort();         // 设置为BEST_EFFORT可靠性
    qos.durability_volatile(); // 设置为VOLATILE持久性

    // 创建临时订阅者监听机器人状态
    auto robot_status_sub = node->create_subscription<rm_referee_msgs::msg::RobotStatus>(
        "/rm_referee/robot_status",
        qos,
        [&](const rm_referee_msgs::msg::RobotStatus::SharedPtr msg)
        {
            robot_id = msg->robot_id;
            RCLCPP_DEBUG(node->get_logger(), "获取到机器人ID: %d", robot_id);
            id_received = true;
        });

    // 等待ID接收或超时 - 修改为主动轮询方式
    {
        const auto start_wait_time = std::chrono::steady_clock::now();
        const auto timeout_duration = std::chrono::seconds(10);

        RCLCPP_INFO(node->get_logger(), "等待接收机器人ID，最多等待10秒...");

        while (!id_received)
        {
            // 关键修改: 处理待处理的ROS消息
            rclcpp::spin_some(node);

            // 检查是否已超时
            auto elapsed = std::chrono::steady_clock::now() - start_wait_time;
            if (elapsed > timeout_duration)
            {
                // 强制使用 ID 7 (红方)，防止 ID 为 0 导致路径点不加载
                robot_id = 7;
                RCLCPP_WARN(node->get_logger(), "超时未收到机器人ID，强制使用默认ID: %d", robot_id);
                break;
            }

            // 短暂等待，避免CPU过载
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (id_received)
        {
            RCLCPP_INFO(node->get_logger(), "成功接收到机器人ID: %d", robot_id);
        }
    }

    // 将ID存储到黑板，供行为树使用
    blackboard->set("robot_id", robot_id);

    if (robot_id == 7)
    {
        RCLCPP_INFO(node->get_logger(), "红方: 加载 %s", waypoints_red_file.c_str());
        // 使用参数路径，而不是硬编码路径
        if (!waypoints_red_file.empty()) {
            bb_manager->load_waypoints(waypoints_red_file);
        } else {
            RCLCPP_ERROR(node->get_logger(), "红方路径点文件参数未设置！");
        }
    }
    else if (robot_id == 107)
    {
        RCLCPP_WARN(node->get_logger(), "蓝方: 加载 %s", waypoints_blue_file.c_str());
        // 使用参数路径
        if (!waypoints_blue_file.empty()) {
            bb_manager->load_waypoints(waypoints_blue_file);
        } else {
            RCLCPP_ERROR(node->get_logger(), "蓝方路径点文件参数未设置！");
        }
    }
    // 在确定颜色并加载路径点后，发布初始位姿
    if (robot_id == 7 || robot_id == 107)
    {
        // [修复] 创建初始位姿发布者：使用标准 Nav2 话题 /initialpose 和带协方差的消息类型
        auto initial_pose_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // 等待一段时间确保发布者已注册
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

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
        // 如果没有指定的初始位姿，尝试使用第一个路径点
        else if (blackboard->get("waypoint_0", init_pose))
        {
            pose_found = true;
            RCLCPP_INFO(node->get_logger(), "使用第一个路径点作为初始位姿");
        }
        // 如果都没有，尝试使用起点
        else if (blackboard->get("waypoint_start", init_pose))
        {
            pose_found = true;
            RCLCPP_INFO(node->get_logger(), "使用起点作为初始位姿");
        }

        if (pose_found)
        {
            // [修复] 转换为 PoseWithCovarianceStamped 并设置协方差
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

            // 在黑板中记录已发布初始位姿
            blackboard->set("initial_pose_published", true);
            // 这里可以继续存 PoseStamped 以便内部逻辑使用
            blackboard->set("initial_pose", init_pose);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "未能找到有效的初始位姿信息");
        }
    }

    try
    {
        auto tree = factory.createTreeFromFile(bt_xml_filename, blackboard);

        auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree);
        RCLCPP_INFO(node->get_logger(), "Groot ZMQ Publisher started. Port: 1666");
        
        RCLCPP_INFO(node->get_logger(), "运行导航行为树");
        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);
            tree.rootNode()->executeTick();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
