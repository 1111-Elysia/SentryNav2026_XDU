// STD
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <vector>
#include <unistd.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

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
#include "sentry_nav_bt_test/patrol_nodes.hpp"
#include "sentry_nav_bt_test/reliable_navigate_to_pose.hpp"
#include "sentry_nav_bt_test/publish_vw_action.hpp"
#include "sentry_nav_bt_test/runtime_config.hpp"

namespace
{

struct BtMessageLogPaths
{
    std::string history_log_path;
    std::string run_log_path;
};

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

std::string getCurrentTimestampString()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);

    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", &local_tm);
    return std::string(buffer);
}

std::string resolveBtMessageLogBaseFilePath(const std::string &configured_path)
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

    return (directory_path / (file_stem + extension)).string();
}

BtMessageLogPaths resolveBtMessageLogPaths(const std::string &configured_path)
{
    namespace fs = std::filesystem;

    BtMessageLogPaths paths;
    if (configured_path.empty()) {
        return paths;
    }

    paths.history_log_path = resolveBtMessageLogBaseFilePath(configured_path);

    const fs::path history_path(paths.history_log_path);
    const std::string file_stem = history_path.stem().string();
    const std::string extension = history_path.extension().string();
    const fs::path dated_log_path = history_path.parent_path() /
        (file_stem + "_" + getCurrentTimestampString() + "_pid" +
         std::to_string(static_cast<long long>(::getpid())) + extension);
    paths.run_log_path = dated_log_path.string();

    return paths;
}

std::string buildBtMessageLogSessionTag()
{
    return getCurrentTimestampString() + " pid=" +
           std::to_string(static_cast<long long>(::getpid()));
}

std::string resolveBtMessageLogProcessLabel(const char *argv0)
{
    namespace fs = std::filesystem;

    if (argv0 == nullptr) {
        return "";
    }

    const std::string executable_name = fs::path(argv0).filename().string();
    if (executable_name.empty()) {
        return "";
    }

    return executable_name + "-1";
}

bool initializeBtMessageLogFile(
    const std::string &log_file_path,
    const rclcpp::Logger &logger,
    const std::string &session_tag)
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
    if (!session_tag.empty()) {
        ofs << "\n# session_start " << session_tag << '\n';
    }

    return true;
}

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
        return std::make_unique<sentry_nav_bt_test::ReliableNavigateToPose>(name, config, node);
    };
    factory.registerBuilder<sentry_nav_bt_test::ReliableNavigateToPose>(
        "ReliableNavigateToPose", reliable_navigate_builder);
    factory.registerNodeType<WaitAction>("Wait");
    // 打印黑板值
    BT::NodeBuilder print_blackboard_builder =
        [](const std::string &name, const BT::NodeConfig &config)
    {
        return std::make_unique<sentry_nav_bt_test::PrintBlackboardValue>(name, config);
    };

    // 注册裁判系统交互节点
    factory.registerNodeType<sentry_nav_bt_test::MaintainSentryPosture>("MaintainSentryPosture");
    factory.registerNodeType<sentry_nav_bt_test::ResolveSentryPosture>("ResolveSentryPosture");
    factory.registerNodeType<sentry_nav_bt_test::ConfirmResurrection>("ConfirmResurrection");
    factory.registerNodeType<sentry_nav_bt_test::BuySentryProjectile>("BuySentryProjectile");
    factory.registerNodeType<sentry_nav_bt_test::EngageRune>("EngageRune");
    factory.registerNodeType<sentry_nav_bt_test::EngageOutpost>("EngageOutpost");
    factory.registerNodeType<sentry_nav_bt_test::PublishVw>("PublishVw");
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
    factory.registerNodeType<sentry_nav_bt_test::PublishControllerName>("PublishControllerName");
    // 打印节点
    factory.registerNodeType<sentry_nav_bt_test::PrintNode>("PrintNode");
    // 设置黑板值
    factory.registerNodeType<sentry_nav_bt_test::SetBlackboardValue>("SetBlackboardValue");

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sentry_nav_bt_test");

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
    std::string bt_message_log_file;
    std::string bt_subtree_dir;
    bool validate_bt_only = false;

    node->declare_parameter("waypoints_file", "");
    node->declare_parameter("bt_message_log_file", "/tmp/sentry_nav_bt_messages.log");
    node->declare_parameter("bt_subtree_dir", "");
    node->declare_parameter("validate_bt_only", false);

    node->get_parameter("waypoints_file", waypoints_file);
    node->get_parameter("bt_message_log_file", bt_message_log_file);
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
    blackboard->set(
        "bt_message_log_process_label",
        resolveBtMessageLogProcessLabel(argv[0]));

    if (!bt_message_log_file.empty())
    {
        const auto bt_message_log_paths = resolveBtMessageLogPaths(bt_message_log_file);
        const std::string session_tag = buildBtMessageLogSessionTag();
        std::string bt_message_log_history_file;

        if (!bt_message_log_paths.history_log_path.empty() &&
            initializeBtMessageLogFile(
                bt_message_log_paths.history_log_path, node->get_logger(), session_tag))
        {
            bt_message_log_history_file = bt_message_log_paths.history_log_path;
            RCLCPP_INFO(
                node->get_logger(),
                "行为树消息累计日志文件: %s",
                bt_message_log_history_file.c_str());
        }

        if (!bt_message_log_paths.run_log_path.empty() &&
            initializeBtMessageLogFile(
                bt_message_log_paths.run_log_path, node->get_logger(), session_tag))
        {
            bt_message_log_file = bt_message_log_paths.run_log_path;
            RCLCPP_INFO(
                node->get_logger(),
                "行为树消息本次运行日志文件: %s",
                bt_message_log_file.c_str());
        } else {
            bt_message_log_file.clear();
        }

        blackboard->set("bt_message_log_history_file", bt_message_log_history_file);
    }

    blackboard->set("bt_message_log_file", bt_message_log_file);

    // 创建黑板管理器
    auto bb_manager = std::make_shared<sentry_nav_bt_test::BlackboardManager>(node, blackboard);

    bb_manager->bb_manager_init();

    if (waypoints_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "路径点文件参数 'waypoints_file' 未设置！");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "加载路径点: %s", waypoints_file.c_str());
    bb_manager->load_waypoints(waypoints_file);

    auto runtime_config = std::make_shared<sentry_nav_bt_test::RuntimeConfigManager>(
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

        // 在黑板中记录已发布初始位姿
        blackboard->set("initial_pose_published", true);
        // 这里可以继续存 PoseStamped 以便内部逻辑使用
        blackboard->set("initial_pose", init_pose);
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
