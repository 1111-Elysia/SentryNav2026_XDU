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
        // 参数
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.117);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("base_link_to_livox_roll", 0.0);
        this->declare_parameter<double>("base_link_to_livox_pitch", 0.0);
        this->declare_parameter<double>("base_link_to_livox_yaw", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);

        this->declare_parameter<double>("a_collect_duration_sec", 5.0);
        this->declare_parameter<int>("a_min_samples", 30);

        this->declare_parameter<std::string>("map_frame_for_D", "map_reloc");
        this->declare_parameter<std::string>("map_frame_for_A", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");

        this->declare_parameter<std::string>("confidence_topic", "/confidence_lightninglm");
        this->declare_parameter<double>("confidence_threshold", 2.0);
        this->declare_parameter<double>("confidence_time_tolerance_sec", 1.0);
        this->declare_parameter<double>("confidence_buffer_keep_sec", 2.0);

        // 读取参数
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);

        double roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0;
        this->get_parameter("base_link_to_livox_roll", roll_deg);
        this->get_parameter("base_link_to_livox_pitch", pitch_deg);
        this->get_parameter("base_link_to_livox_yaw", yaw_deg);

        livox_offset_roll_ = roll_deg*M_PI/180.0;
        livox_offset_pitch_ = pitch_deg*M_PI/180.0;
        livox_offset_yaw_ = yaw_deg*M_PI/180.0;

        double publish_rate=100.0;
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

        // 计算 C: base_link -> livox_frame
        tf_C_.setOrigin(tf2::Vector3(livox_offset_x_, livox_offset_y_, livox_offset_z_));
        tf2::Quaternion q_C;
        q_C.setRPY(livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_);
        tf_C_.setRotation(q_C);
        tf_C_inv_ = tf_C_.inverse();

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        confidence_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            confidence_topic_,
            rclcpp::QoS(rclcpp::KeepLast(300)).best_effort().durability_volatile(),
            std::bind(&TfOdomPublisher::confidenceCallback, this, std::placeholders::_1));

        lio_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio/robo/odom",
            rclcpp::QoS(rclcpp::KeepLast(200)).best_effort().durability_volatile(),
            std::bind(&TfOdomPublisher::lioOdomCallback, this, std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0/publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));

        publishStaticTransform();

        // 修复：添加了缺失的参数
        RCLCPP_INFO(this->get_logger(),
            "节点启动: 初始收集 %.2fs 的 D 用于计算 map->odom, 后续用卡尔曼检测收敛一次性更新", 
            a_collect_duration_sec_);
    }

private:
    struct ConfidenceSample {
        rclcpp::Time stamp;
        double confidence = 0.0;
    };

    // ------------------ 卡尔曼滤波状态 ------------------
    struct KalmanState {
        double x=0, y=0, yaw=0;
        double Px=1e-4, Py=1e-4, Pyaw=1e-4; // 协方差
    };
    KalmanState kalman_;
    bool kalman_initialized_ = false;
    bool final_A_published_ = false;

    double process_noise_ = 1e-6;
    double obs_noise_ = 1e-3;
    double convergence_thresh_pos_ = 0.01;   // 1cm
    double convergence_thresh_yaw_ = 0.0087;  // 0.5 deg
    int convergence_count_ = 0;
    int convergence_required_ = 5;

    // missing members decl
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr confidence_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    double livox_offset_x_, livox_offset_y_, livox_offset_z_;
    double livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_;
    double a_collect_duration_sec_;
    int a_min_samples_;
    std::string map_frame_for_D_, map_frame_for_A_, odom_frame_;
    std::string confidence_topic_;
    double confidence_threshold_;
    double confidence_time_tolerance_sec_;
    double confidence_buffer_keep_sec_;

    tf2::Transform tf_C_, tf_C_inv_;
    
    // 修复：添加 mutable 关键字，允许在 const 成员函数中加锁
    mutable std::mutex conf_mtx_;
    std::deque<ConfidenceSample> conf_buf_;
    
    nav_msgs::msg::Odometry last_lio_odom_;
    bool have_lio_odom_ = false;
    
    std::vector<tf2::Transform> d_samples_;
    // ------------------------------------------------

    static tf2::Quaternion averageQuaternionHemisphereSafe(const std::vector<tf2::Quaternion> &qs)
    {
        tf2::Quaternion q_avg(0,0,0,0);
        if (qs.empty()){ q_avg.setRPY(0,0,0); return q_avg;}
        tf2::Quaternion q_ref = qs.front(); q_ref.normalize();
        double sx=0, sy=0, sz=0, sw=0;
        for(auto q: qs){
            q.normalize();
            double dot = q_ref.x()*q.x()+q_ref.y()*q.y()+q_ref.z()*q.z()+q_ref.w()*q.w();
            if(dot<0) q = tf2::Quaternion(-q.x(),-q.y(),-q.z(),-q.w());
            sx+=q.x(); sy+=q.y(); sz+=q.z(); sw+=q.w();
        }
        q_avg = tf2::Quaternion(sx,sy,sz,sw);
        q_avg.normalize();
        return q_avg;
    }

    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = rclcpp::Time(0);
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "livox_frame";
        tf_msg.transform = tf2::toMsg(tf_C_);
        static_tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(), "静态变换已发布: base_link -> livox_frame");
    }

    void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(!msg) return;
        last_lio_odom_ = *msg;
        have_lio_odom_ = true;
    }

    void confidenceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if(!msg || msg->data.size()<2) return;
        double timestamp_sec = msg->data[0];
        double conf = msg->data[1];
        int64_t ns = static_cast<int64_t>(timestamp_sec*1e9);
        rclcpp::Time stamp(ns, this->get_clock()->get_clock_type());
        std::lock_guard<std::mutex> lk(conf_mtx_);
        conf_buf_.push_back({stamp,conf});
        rclcpp::Time now = this->now();
        while(!conf_buf_.empty()){
            if((now-conf_buf_.front().stamp).seconds()<=confidence_buffer_keep_sec_) break;
            conf_buf_.pop_front();
        }
    }

    bool findConfidenceNearStamp(const rclcpp::Time &d_stamp, double &out_conf) const
    {
        std::lock_guard<std::mutex> lk(conf_mtx_);
        if(conf_buf_.empty()) return false;
        bool found=false;
        double best_dt = std::numeric_limits<double>::infinity();
        double best_conf=0;
        for(const auto &s: conf_buf_){
            double dt = std::fabs((s.stamp-d_stamp).seconds());
            if(dt <= confidence_time_tolerance_sec_ && dt<best_dt){
                best_dt=dt; best_conf=s.confidence; found=true;
            }
        }
        if(found) out_conf=best_conf;
        return found;
    }

    void timerCallback()
    {
        rclcpp::Time now = this->now();
        if(final_A_published_) {
            // A 已经收敛发布过，直接发布 B / odom
            publishBAndOdom(now);
            return;
        }

        // 1) 收集 D
        geometry_msgs::msg::TransformStamped D_msg;
        bool got_D=false;
        try{
            D_msg = tf_buffer_.lookupTransform(map_frame_for_D_, "livox_frame_reloc", rclcpp::Time(0));
            got_D = true;
        }catch(const tf2::TransformException &ex){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "等待 D(%s->livox_frame_reloc): %s", map_frame_for_D_.c_str(), ex.what());
        }

        if(got_D && D_msg.header.frame_id==map_frame_for_D_){
            rclcpp::Time d_stamp(D_msg.header.stamp,this->get_clock()->get_clock_type());
            if(D_msg.header.stamp.sec==0 && D_msg.header.stamp.nanosec==0) d_stamp=now;
            double conf=0;
            bool have_conf = findConfidenceNearStamp(d_stamp,conf);
            if(!have_conf || conf<=confidence_threshold_){
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "丢弃 D: conf=%.3f",conf);
            }else{
                tf2::Transform tf_D; tf2::fromMsg(D_msg.transform,tf_D);
                d_samples_.push_back(tf_D);
                // 关键更改：保持 d_samples_ 为滑动窗口，防止一直积累旧数据，
                // 这样 Kalman 滤波器才能响应最新的定位变化。
                if(d_samples_.size() > (size_t)(a_min_samples_ * 2)) {
                    d_samples_.erase(d_samples_.begin());
                }
            }
        }

        // 2) 初始发布 或 卡尔曼检测收敛
        if(d_samples_.size() >= static_cast<size_t>(a_min_samples_)){
            tf2::Transform tf_D_avg;
            tf2::Vector3 sum_t(0,0,0);
            std::vector<tf2::Quaternion> qs;
            for(const auto &t: d_samples_){
                sum_t += t.getOrigin();
                qs.push_back(t.getRotation());
            }
            tf2::Vector3 mean_t = sum_t*(1.0/d_samples_.size());
            tf2::Quaternion mean_q = averageQuaternionHemisphereSafe(qs);
            tf_D_avg.setOrigin(mean_t);
            tf_D_avg.setRotation(mean_q);

            // A_raw = D_avg * C_inv
            tf2::Transform tf_A_raw = tf_D_avg * tf_C_inv_;
            tf2::Vector3 t = tf_A_raw.getOrigin();
            t.setZ(0.0);
            tf_A_raw.setOrigin(t);
            double roll,pitch,yaw;
            tf2::Matrix3x3(tf_A_raw.getRotation()).getRPY(roll,pitch,yaw);
            tf2::Quaternion q_yaw; q_yaw.setRPY(0,0,yaw); tf_A_raw.setRotation(q_yaw);

            double obs_x = t.x(), obs_y=t.y(), obs_yaw=yaw;

            // 卡尔曼初始化
            if(!kalman_initialized_){
                kalman_.x=obs_x; kalman_.y=obs_y; kalman_.yaw=obs_yaw;
                kalman_initialized_=true;

                // 发布初始静态 TF
                geometry_msgs::msg::TransformStamped A_msg;
                A_msg.header.stamp = rclcpp::Time(0);
                A_msg.header.frame_id = map_frame_for_A_;
                A_msg.child_frame_id = odom_frame_;
                A_msg.transform = tf2::toMsg(tf_A_raw);
                static_tf_broadcaster_->sendTransform(A_msg);
                RCLCPP_INFO(this->get_logger(), "初始 A 发布完成 (Rough init)");
            }else{
                // 卡尔曼滤波更新
                kalmanUpdate(obs_x, obs_y, obs_yaw);

                // 收敛检测 (使用滤波器状态与当前观测值的差异)
                double dx = std::fabs(kalman_.x - obs_x);
                double dy = std::fabs(kalman_.y - obs_y);
                double dyaw = std::fabs(normalizeAngle(kalman_.yaw - obs_yaw));
                
                // 只有当观测值稳定（d_samples_平均）且与Kalman估计一致时判定为收敛
                if(dx<convergence_thresh_pos_ && dy<convergence_thresh_pos_ && dyaw<convergence_thresh_yaw_){
                    convergence_count_++;
                }else{
                    convergence_count_=0;
                }

                if(convergence_count_>=convergence_required_ && !final_A_published_){
                    // 发布最终收敛 A
                    tf2::Transform tf_final;
                    tf_final.setOrigin(tf2::Vector3(kalman_.x, kalman_.y, 0));
                    tf2::Quaternion q_final; q_final.setRPY(0,0,kalman_.yaw);
                    tf_final.setRotation(q_final);
                    geometry_msgs::msg::TransformStamped A_msg;
                    A_msg.header.stamp = rclcpp::Time(0);
                    A_msg.header.frame_id = map_frame_for_A_;
                    A_msg.child_frame_id = odom_frame_;
                    A_msg.transform = tf2::toMsg(tf_final);
                    static_tf_broadcaster_->sendTransform(A_msg);
                    RCLCPP_INFO(this->get_logger(), "最终收敛 A 发布完成 (Converged)");
                    final_A_published_ = true;
                    d_samples_.clear();
                }
            }
        }

        // 3) 发布 B / odom
        publishBAndOdom(now);
    }

    void kalmanUpdate(double obs_x, double obs_y, double obs_yaw)
    {
        // 预测（简单恒定状态）
        kalman_.Px += process_noise_; kalman_.Py += process_noise_; kalman_.Pyaw += process_noise_;

        // 卡尔曼增益
        double Kx = kalman_.Px / (kalman_.Px + obs_noise_);
        double Ky = kalman_.Py / (kalman_.Py + obs_noise_);
        double Kyaw = kalman_.Pyaw / (kalman_.Pyaw + obs_noise_);

        // 更新状态
        kalman_.x += Kx * (obs_x - kalman_.x);
        kalman_.y += Ky * (obs_y - kalman_.y);
        kalman_.yaw += Kyaw * normalizeAngle(obs_yaw - kalman_.yaw);

        // 更新协方差
        kalman_.Px = (1-Kx)*kalman_.Px;
        kalman_.Py = (1-Ky)*kalman_.Py;
        kalman_.Pyaw = (1-Kyaw)*kalman_.Pyaw;
    }

    double normalizeAngle(double a)
    {
        while(a>M_PI) a-=2*M_PI;
        while(a<-M_PI) a+=2*M_PI;
        return a;
    }

    void publishBAndOdom(const rclcpp::Time &now)
    {
        if(!have_lio_odom_) return;
        double age = (now - last_lio_odom_.header.stamp).seconds();
        if(age>5.0) return;

        tf2::Transform tf_E;
        tf_E.setOrigin(tf2::Vector3(
            last_lio_odom_.pose.pose.position.x,
            last_lio_odom_.pose.pose.position.y,
            last_lio_odom_.pose.pose.position.z
        ));
        tf2::Quaternion qE(
            last_lio_odom_.pose.pose.orientation.x,
            last_lio_odom_.pose.pose.orientation.y,
            last_lio_odom_.pose.pose.orientation.z,
            last_lio_odom_.pose.pose.orientation.w
        );
        qE.normalize(); tf_E.setRotation(qE);

        tf2::Transform tf_B = tf_E * tf_C_inv_;

        geometry_msgs::msg::TransformStamped B_msg;
        B_msg.header.stamp = last_lio_odom_.header.stamp;
        B_msg.header.frame_id = odom_frame_; // using param
        B_msg.child_frame_id = "base_link";
        B_msg.transform = tf2::toMsg(tf_B);
        tf_broadcaster_->sendTransform(B_msg);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = B_msg.header.stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = "base_link";
        // Convert tf_B back to pose for the message
        tf2::toMsg(tf_B, odom_msg.pose.pose);
        
        // Use twist from LIO odom (assuming it's in base_link frame, child_frame_id matches)
        odom_msg.twist = last_lio_odom_.twist;

        // Set covariance if needed (copying from LIO or default)
        odom_msg.pose.covariance = last_lio_odom_.pose.covariance;
        odom_msg.twist.covariance = last_lio_odom_.twist.covariance;

        odom_publisher_->publish(odom_msg);
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}