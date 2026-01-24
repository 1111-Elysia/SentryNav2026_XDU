#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher()
        : Node("tf_odom_publisher"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_, this, true) // 独立线程处理 /tf
    {
        // 参数：base_link → livox_frame 的变换 C
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.117);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("base_link_to_livox_roll", 0.0);
        this->declare_parameter<double>("base_link_to_livox_pitch", 0.0);
        this->declare_parameter<double>("base_link_to_livox_yaw", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);

        // A 静态化相关参数
        this->declare_parameter<double>("a_collect_duration_sec", 5.0);
        this->declare_parameter<int>("a_min_samples", 30);

        // frame 名称参数
        this->declare_parameter<std::string>("map_frame_for_D", "map_reloc");
        this->declare_parameter<std::string>("map_frame_for_A", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");

        // 新增：置信度话题与匹配参数
        this->declare_parameter<std::string>("confidence_topic", "/confidence_lightninglm");
        this->declare_parameter<double>("confidence_threshold", 2.0);
        this->declare_parameter<double>("confidence_time_tolerance_sec", 1.0); // D_stamp ±1.0s
        this->declare_parameter<double>("confidence_buffer_keep_sec", 2.0);     // 缓存最近2s

        // 读取参数
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);

        double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
        this->get_parameter("base_link_to_livox_roll", roll_deg);
        this->get_parameter("base_link_to_livox_pitch", pitch_deg);
        this->get_parameter("base_link_to_livox_yaw", yaw_deg);

        livox_offset_roll_  = roll_deg * M_PI / 180.0;
        livox_offset_pitch_ = pitch_deg * M_PI / 180.0;
        livox_offset_yaw_   = yaw_deg * M_PI / 180.0;

        double publish_rate = 100.0;
        this->get_parameter("publish_rate", publish_rate);

        this->get_parameter("a_collect_duration_sec", a_collect_duration_sec_);
        this->get_parameter("a_min_samples", a_min_samples_);
        this->get_parameter("map_frame_for_D", map_frame_for_D_);
        this->get_parameter("map_frame_for_A", map_frame_for_A_);
        this->get_parameter("odom_frame", odom_frame_);

        this->get_parameter("confidence_topic", confidence_topic_);
        this->get_parameter("confidence_threshold", confidence_threshold_);
        this->get_parameter("confidence_time_tolerance_sec", confidence_time_tolerance_sec_);
        this->get_parameter("confidence_buffer_keep_sec", confidence_buffer_keep_sec_);

        // 计算 C: base_link → livox_frame
        tf_C_.setOrigin(tf2::Vector3(livox_offset_x_, livox_offset_y_, livox_offset_z_));
        tf2::Quaternion q_C;
        q_C.setRPY(livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_);
        tf_C_.setRotation(q_C);
        tf_C_inv_ = tf_C_.inverse();

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        // 监听 /confidence_lightninglm: Float64MultiArray
        // data[0] = timestamp(sec), data[1] = confidence
        confidence_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            confidence_topic_,
            rclcpp::QoS(rclcpp::KeepLast(300)).best_effort().durability_volatile(),
            std::bind(&TfOdomPublisher::confidenceCallback, this, std::placeholders::_1));

        // 新增：订阅 /lio/robo/odom 用于构造 E(odom->livox_frame_two)
        lio_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio/robo/odom",
            rclcpp::QoS(rclcpp::KeepLast(200)).best_effort().durability_volatile(),
            std::bind(&TfOdomPublisher::lioOdomCallback, this, std::placeholders::_1));
        
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));

        publishStaticTransform();

        RCLCPP_INFO(this->get_logger(),
                    "A(map->odom) 启动后收集 %.2fs 的 D(%s->livox_frame_reloc)，仅当匹配到 %s 且 confidence>%.3f（|dt|<=%.3fs）才纳入平均。",
                    a_collect_duration_sec_, map_frame_for_D_.c_str(),
                    confidence_topic_.c_str(), confidence_threshold_, confidence_time_tolerance_sec_);
    }

private:
    struct ConfidenceSample {
        rclcpp::Time stamp;
        double confidence = 0.0;
    };

    static tf2::Quaternion averageQuaternionHemisphereSafe(const std::vector<tf2::Quaternion> &qs)
    {
        tf2::Quaternion q_avg(0, 0, 0, 0);
        if (qs.empty()) {
            q_avg.setRPY(0, 0, 0);
            return q_avg;
        }

        tf2::Quaternion q_ref = qs.front();
        q_ref.normalize();

        double sx = 0.0, sy = 0.0, sz = 0.0, sw = 0.0;
        for (auto q : qs) {
            q.normalize();
            const double dot = q_ref.x() * q.x() + q_ref.y() * q.y() + q_ref.z() * q.z() + q_ref.w() * q.w();
            if (dot < 0.0) {
                q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
            }
            sx += q.x();
            sy += q.y();
            sz += q.z();
            sw += q.w();
        }

        q_avg = tf2::Quaternion(sx, sy, sz, sw);
        q_avg.normalize();
        return q_avg;
    }

    void publishStaticTransform()
    {
        // 发布静态 TF: C (base_link → livox_frame)
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = rclcpp::Time(0);
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "livox_frame";
        tf_msg.transform = tf2::toMsg(tf_C_);
        static_tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(), "静态变换已发布: base_link -> livox_frame");
    }

    void publishStaticAFromAvgD(const rclcpp::Time &stamp_now)
    {
        if (static_A_published_) return;

        if (d_samples_.size() < static_cast<size_t>(std::max(1, a_min_samples_))) {
            RCLCPP_WARN(this->get_logger(),
                        "D 有效采样不足，无法发布静态 A：samples=%zu < min=%d",
                        d_samples_.size(), a_min_samples_);
            return;
        }

        // 平移平均
        tf2::Vector3 sum_t(0, 0, 0);
        std::vector<tf2::Quaternion> qs;
        qs.reserve(d_samples_.size());
        for (const auto &t : d_samples_) {
            sum_t += t.getOrigin();
            qs.push_back(t.getRotation());
        }

        const tf2::Vector3 mean_t = sum_t * (1.0 / static_cast<double>(d_samples_.size()));
        const tf2::Quaternion mean_q = averageQuaternionHemisphereSafe(qs);

        // 平移 + 旋转构造 D_avg
        tf2::Transform tf_D_avg;
        tf_D_avg.setOrigin(mean_t);
        tf_D_avg.setRotation(mean_q);

        // A_raw = D_avg * C_inv
        tf2::Transform tf_A_raw = tf_D_avg * tf_C_inv_;

        // =============================
        // 修正逻辑：Z 固定，roll/pitch=0，只保留 yaw
        // =============================
        tf2::Transform tf_A = tf_A_raw;

        // 平移 Z 固定
        tf2::Vector3 t = tf_A.getOrigin();
        t.setZ(0.0);
        tf_A.setOrigin(t);

        // 旋转只保留 yaw
        tf2::Quaternion q = tf_A.getRotation();
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);
        tf_A.setRotation(q_yaw);
        // =============================

        // 发布静态 A
        geometry_msgs::msg::TransformStamped A_msg;
        A_msg.header.stamp = rclcpp::Time(0);
        A_msg.header.frame_id = map_frame_for_A_;
        A_msg.child_frame_id = odom_frame_;
        A_msg.transform = tf2::toMsg(tf_A);

        static_tf_broadcaster_->sendTransform(A_msg);
        static_A_published_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "已发布静态 A(%s->%s)，有效采样数=%zu。",
                    map_frame_for_A_.c_str(), odom_frame_.c_str(), d_samples_.size());
    }


    void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg) return;
        last_lio_odom_ = *msg;
        have_lio_odom_ = true;
    }


    void confidenceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!msg || msg->data.size() < 2) return;

        const double timestamp_sec = msg->data[0];
        const double conf = msg->data[1];

        const int64_t ns = static_cast<int64_t>(timestamp_sec * 1e9);
        const rclcpp::Time stamp(ns, this->get_clock()->get_clock_type());

        std::lock_guard<std::mutex> lk(conf_mtx_);
        conf_buf_.push_back(ConfidenceSample{stamp, conf});

        // 裁剪缓存
        const rclcpp::Time now = this->now();
        while (!conf_buf_.empty()) {
            const double age = (now - conf_buf_.front().stamp).seconds();
            if (age <= confidence_buffer_keep_sec_) break;
            conf_buf_.pop_front();
        }
    }

    bool findConfidenceNearStamp(const rclcpp::Time &d_stamp, double &out_conf) const
    {
        std::lock_guard<std::mutex> lk(conf_mtx_);
        if (conf_buf_.empty()) return false;

        bool found = false;
        double best_abs_dt = std::numeric_limits<double>::infinity();
        double best_conf = 0.0;

        for (const auto &s : conf_buf_) {
            const double abs_dt = std::fabs((s.stamp - d_stamp).seconds());
            if (abs_dt <= confidence_time_tolerance_sec_ && abs_dt < best_abs_dt) {
                best_abs_dt = abs_dt;
                best_conf = s.confidence;
                found = true;
            }
        }

        if (found) out_conf = best_conf;
        return found;
    }

    void timerCallback()
    {
        rclcpp::Time now = this->now();

        // ==========================
        // 1) 收集 D 用于静态 A
        // ==========================
        if (!static_A_published_) {
            geometry_msgs::msg::TransformStamped D_msg;
            bool got_D = false;

            try {
                D_msg = tf_buffer_.lookupTransform(map_frame_for_D_, "livox_frame_reloc", rclcpp::Time(0));
                got_D = true;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 D(%s->livox_frame_reloc) 用于静态A: %s",
                                     map_frame_for_D_.c_str(), ex.what());
            }

            if (got_D && D_msg.header.frame_id == map_frame_for_D_) {
                if (!d_collecting_) {
                    d_collecting_ = true;
                    d_collect_start_time_ = now;
                    d_samples_.clear();
                    RCLCPP_INFO(this->get_logger(),
                                "开始收集 D(%s->livox_frame_reloc)，持续 %.2fs（confidence过滤）...",
                                map_frame_for_D_.c_str(), a_collect_duration_sec_);
                }

                rclcpp::Time d_stamp(D_msg.header.stamp, this->get_clock()->get_clock_type());
                if (D_msg.header.stamp.sec == 0 && D_msg.header.stamp.nanosec == 0) {
                    // 若 TF 没给有效 stamp，则退化用 now（避免永远匹配不到）
                    d_stamp = now;
                }

                double conf = 0.0;
                const bool have_conf = findConfidenceNearStamp(d_stamp, conf);

                if (!have_conf) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "丢弃 D：±%.3fs 内未找到 %s 匹配（D_stamp=%.3f）",
                                         confidence_time_tolerance_sec_, confidence_topic_.c_str(), d_stamp.seconds());
                } else if (conf <= confidence_threshold_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "丢弃 D：confidence=%.3f <= %.3f（D_stamp=%.3f）",
                                         conf, confidence_threshold_, d_stamp.seconds());
                } else {
                    tf2::Transform tf_D;
                    tf2::fromMsg(D_msg.transform, tf_D);
                    d_samples_.push_back(tf_D);
                }

                const double elapsed = (now - d_collect_start_time_).seconds();
                if (elapsed >= a_collect_duration_sec_) {
                    publishStaticAFromAvgD(now);
                    d_samples_.clear();
                    if (!static_A_published_) {
                        RCLCPP_WARN(this->get_logger(), "采集时间结束但发布失败（采样不足），重置状态以重新开始采集...");
                        d_collecting_ = false; 
                    }
                }
            }
        }

        // ==========================
        // 2) 用 /lio/robo/odom 构造 E(odom->livox_frame_two) → 计算/发布 B 与 /odom
        // ==========================
        bool got_E = false;
        geometry_msgs::msg::TransformStamped E_msg;
        if (!have_lio_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "等待 /lio/robo/odom（nav_msgs/Odometry）以构造 E(odom->livox_frame_two)...");
        } else {
            const double age = (now - last_lio_odom_.header.stamp).seconds();
            if (age > 5.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "/lio/robo/odom 过旧(%.2fs)，暂不更新 B", age);
            } else {
                got_E = true;
            }
        }

        if (!got_E) return;

        try {
            // 构造 E: odom -> livox_frame_two
            tf2::Transform tf_E;
            tf_E.setOrigin(tf2::Vector3(
                last_lio_odom_.pose.pose.position.x,
                last_lio_odom_.pose.pose.position.y,
                last_lio_odom_.pose.pose.position.z));

            tf2::Quaternion qE(
                last_lio_odom_.pose.pose.orientation.x,
                last_lio_odom_.pose.pose.orientation.y,
                last_lio_odom_.pose.pose.orientation.z,
                last_lio_odom_.pose.pose.orientation.w);
            qE.normalize();
            tf_E.setRotation(qE);

            // 兼容检查：期望 child_frame_id 为 livox_frame_two
            // if (last_lio_odom_.child_frame_id != "livox_frame_two") {
            //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            //                          "/lio/robo/odom child_frame_id=%s，期望=livox_frame_two，仍按位姿计算",
            //                          last_lio_odom_.child_frame_id.c_str());
            // }
            // if (last_lio_odom_.header.frame_id != "odom") {
            //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            //                          "/lio/robo/odom header.frame_id=%s，期望=odom",
            //                          last_lio_odom_.header.frame_id.c_str());
            // }

            // 计算 B = E * C_inv
            tf2::Transform tf_B = tf_E * tf_C_inv_;

            // 发布 B(odom->base_link) TF
            geometry_msgs::msg::TransformStamped B_msg;
            B_msg.header.stamp = last_lio_odom_.header.stamp;
            B_msg.header.frame_id = "odom";
            B_msg.child_frame_id = "base_link";
            B_msg.transform = tf2::toMsg(tf_B);
            tf_broadcaster_->sendTransform(B_msg);

            // 发布 /odom（保持原来速度/协方差映射）
            if (have_lio_odom_) {
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = B_msg.header.stamp;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_link";

                odom_msg.pose.pose.position.x = tf_B.getOrigin().x();
                odom_msg.pose.pose.position.y = tf_B.getOrigin().y();
                odom_msg.pose.pose.position.z = tf_B.getOrigin().z();
                odom_msg.pose.pose.orientation = tf2::toMsg(tf_B.getRotation());

                tf2::Vector3 linear_vel(
                    last_lio_odom_.twist.twist.linear.x,
                    last_lio_odom_.twist.twist.linear.y,
                    last_lio_odom_.twist.twist.linear.z);
                tf2::Vector3 angular_vel(
                    last_lio_odom_.twist.twist.angular.x,
                    last_lio_odom_.twist.twist.angular.y,
                    last_lio_odom_.twist.twist.angular.z);

                tf2::Vector3 linear_vel_bl = tf_C_inv_.getBasis() * linear_vel;
                tf2::Vector3 angular_vel_bl = tf_C_inv_.getBasis() * angular_vel;

                odom_msg.twist.twist.linear.x = linear_vel_bl.x();
                odom_msg.twist.twist.linear.y = linear_vel_bl.y();
                odom_msg.twist.twist.linear.z = linear_vel_bl.z();
                odom_msg.twist.twist.angular.x = angular_vel_bl.x();
                odom_msg.twist.twist.angular.y = angular_vel_bl.y();
                odom_msg.twist.twist.angular.z = angular_vel_bl.z();

                odom_msg.pose.covariance = last_lio_odom_.pose.covariance;
                odom_msg.twist.covariance = last_lio_odom_.twist.covariance;

                odom_publisher_->publish(odom_msg);
            }

        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "计算/发布 B 失败: %s", e.what());
        }
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_subscriber_; // 新增
    rclcpp::TimerBase::SharedPtr timer_;

    // C: base_link -> livox_frame
    tf2::Transform tf_C_;
    tf2::Transform tf_C_inv_;
    double livox_offset_x_ = 0.0;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    double livox_offset_roll_ = 0.0;
    double livox_offset_pitch_ = 0.0;
    double livox_offset_yaw_ = 0.0;

    // E 缓存
    geometry_msgs::msg::TransformStamped last_E_msg_;
    bool have_E_ = false;

    // E 来源改为 /lio/robo/odom
    nav_msgs::msg::Odometry last_lio_odom_; // 新增
    bool have_lio_odom_ = false;            // 新增

    // A 静态化：D 的平均
    bool static_A_published_ = false;
    bool d_collecting_ = false;
    rclcpp::Time d_collect_start_time_;
    double a_collect_duration_sec_ = 10.0;
    int a_min_samples_ = 30;
    std::vector<tf2::Transform> d_samples_;

    std::string map_frame_for_D_ = "map_reloc";
    std::string map_frame_for_A_ = "map";
    std::string odom_frame_ = "odom";

    // confidence 缓存与匹配
    std::string confidence_topic_ = "/confidence_lightninglm";
    double confidence_threshold_ = 2.0;
    double confidence_time_tolerance_sec_ = 1.0;
    double confidence_buffer_keep_sec_ = 2.0;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr confidence_subscriber_;
    mutable std::mutex conf_mtx_;
    std::deque<ConfidenceSample> conf_buf_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}