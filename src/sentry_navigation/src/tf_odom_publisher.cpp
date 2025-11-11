#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher() : Node("tf_odom_publisher"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        // ===== 声明参数 =====
        this->declare_parameter<std::string>("map_yaml_path", "");
        this->declare_parameter<bool>("auto_compensate_origin", true);
        this->declare_parameter<double>("base_link_to_livox_x", 0.1);
        this->declare_parameter<double>("base_link_to_livox_y", 0.0);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        
        // ===== 获取参数 =====
        std::string map_yaml_path;
        this->get_parameter("map_yaml_path", map_yaml_path);
        this->get_parameter("auto_compensate_origin", auto_compensate_origin_);
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        
        // ===== 加载地图原点 =====
        if (!map_yaml_path.empty() && std::filesystem::exists(map_yaml_path)) {
            loadMapOrigin(map_yaml_path);
        } else if (auto_compensate_origin_) {
            RCLCPP_WARN(this->get_logger(), 
                "地图 YAML 路径未指定或文件不存在,禁用原点补偿");
            auto_compensate_origin_ = false;
        }
        
        // ===== 初始化发布器和订阅器 =====
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&TfOdomPublisher::odometryCallback, this, std::placeholders::_1));
        
        tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            std::bind(&TfOdomPublisher::tfCallback, this, std::placeholders::_1));
        
        // ===== 定时检查重定位状态 =====
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TfOdomPublisher::checkRelocalizationStatus, this));
        
        // ===== 发布静态变换 =====
        publishStaticTransform();
        
        RCLCPP_INFO(this->get_logger(), "TF Odom Publisher 初始化完成");
        RCLCPP_INFO(this->get_logger(), "  地图原点偏移: [%.2f, %.2f]", 
                    map_origin_x_, map_origin_y_);
        RCLCPP_INFO(this->get_logger(), "  自动补偿: %s", 
                    auto_compensate_origin_ ? "启用" : "禁用");
    }

private:
    // ===== 加载地图原点 =====
    void loadMapOrigin(const std::string& yaml_path)
    {
        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            auto origin = config["origin"].as<std::vector<double>>();
            
            map_origin_x_ = origin[0];
            map_origin_y_ = origin[1];
            map_origin_loaded_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                "成功加载地图原点: [%.2f, %.2f]", map_origin_x_, map_origin_y_);
            
            // ===== 判断是否需要补偿 =====
            if (std::abs(map_origin_x_) > 0.05 || std::abs(map_origin_y_) > 0.05) {
                RCLCPP_WARN(this->get_logger(), 
                    "⚠️ 地图原点偏移较大,将启用自动补偿");
                needs_compensation_ = true;
            } else {
                RCLCPP_INFO(this->get_logger(), 
                    "✓ 地图原点接近(0,0),无需补偿");
                needs_compensation_ = false;
                auto_compensate_origin_ = false;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "加载地图 YAML 失败: %s", e.what());
            map_origin_loaded_ = false;
            auto_compensate_origin_ = false;
        }
    }
    
    // ===== 发布静态变换: base_link → livox_frame =====
    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "livox_frame";
        
        static_transform.transform.translation.x = livox_offset_x_;
        static_transform.transform.translation.y = livox_offset_y_;
        static_transform.transform.translation.z = livox_offset_z_;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(static_transform);
        
        RCLCPP_INFO(this->get_logger(), 
            "发布静态 TF: base_link → livox_frame [%.2f, %.2f, %.2f]",
            livox_offset_x_, livox_offset_y_, livox_offset_z_);
    }

    // ===== 检查重定位状态 =====
    void checkRelocalizationStatus()
    {
        if (is_relocalized_) return;
        
        try {
            // 尝试查询 map→odom 变换
            auto transform = tf_buffer_.lookupTransform(
                "map", "odom", tf2::TimePointZero);
            
            // 检查变换是否有效(非单位变换)
            double tx = transform.transform.translation.x;
            double ty = transform.transform.translation.y;
            
            if (std::abs(tx) > 0.01 || std::abs(ty) > 0.01) {
                is_relocalized_ = true;
                RCLCPP_INFO(this->get_logger(), 
                    "✓ 检测到重定位成功,禁用原点补偿");
                RCLCPP_INFO(this->get_logger(), 
                    "  map→odom: [%.2f, %.2f]", tx, ty);
            }
            
        } catch (const tf2::TransformException& ex) {
            // map→odom 还未发布,继续等待
        }
    }

    // ===== 里程计回调 =====
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // ===== 1. 转发里程计话题 =====
        auto odom_msg = *msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_publisher_->publish(odom_msg);
        
        // ===== 2. 提取位姿信息 =====
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        auto q = msg->pose.pose.orientation;
        
        // ===== 3. 构建 odom→base_link 变换 =====
        geometry_msgs::msg::TransformStamped odom_to_baselink;
        odom_to_baselink.header.stamp = msg->header.stamp;
        odom_to_baselink.header.frame_id = "odom";
        odom_to_baselink.child_frame_id = "base_link";
        
        // ===== 4. 根据重定位状态决定是否补偿 =====
        if (!is_relocalized_ && auto_compensate_origin_ && 
            needs_compensation_ && map_origin_loaded_) {
            // 未重定位且需要补偿:减去地图原点偏移
            odom_to_baselink.transform.translation.x = x - map_origin_x_;
            odom_to_baselink.transform.translation.y = y - map_origin_y_;
            odom_to_baselink.transform.translation.z = z;
            
            // 每秒输出一次调试信息
            auto now = this->now();
            if ((now - last_log_time_).seconds() > 1.0) {
                RCLCPP_INFO(this->get_logger(), 
                    "补偿模式: Fast-LIO [%.2f, %.2f] → base_link [%.2f, %.2f]",
                    x, y, 
                    x - map_origin_x_, y - map_origin_y_);
                last_log_time_ = now;
            }
        } else {
            // 已重定位或无需补偿:直接使用里程计数据
            odom_to_baselink.transform.translation.x = x;
            odom_to_baselink.transform.translation.y = y;
            odom_to_baselink.transform.translation.z = z;
        }
        
        odom_to_baselink.transform.rotation = q;
        
        // ===== 5. 发布 TF =====
        tf_broadcaster_->sendTransform(odom_to_baselink);
    }

    // ===== TF 消息回调(监听 map→odom) =====
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            if (transform.header.frame_id == "map" && 
                transform.child_frame_id == "odom") {
                
                if (!is_relocalized_) {
                    double tx = transform.transform.translation.x;
                    double ty = transform.transform.translation.y;
                    
                    if (std::abs(tx) > 0.01 || std::abs(ty) > 0.01) {
                        is_relocalized_ = true;
                        RCLCPP_INFO(this->get_logger(), 
                            "✓ 通过 /tf 检测到重定位: map→odom [%.2f, %.2f]",
                            tx, ty);
                    }
                }
                break;
            }
        }
    }

private:
    // ===== TF 相关 =====
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ===== 发布器和订阅器 =====
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    
    // ===== 定时器 =====
    rclcpp::TimerBase::SharedPtr check_timer_;
    
    // ===== 状态变量 =====
    bool is_relocalized_ = false;           // 是否已重定位
    bool map_origin_loaded_ = false;        // 地图原点是否加载成功
    bool needs_compensation_ = false;       // 是否需要补偿
    bool auto_compensate_origin_ = true;    // 是否启用自动补偿
    
    // ===== 地图原点偏移 =====
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    
    // ===== 雷达偏移 =====
    double livox_offset_x_ = 0.1;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    
    // ===== 日志控制 =====
    rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}