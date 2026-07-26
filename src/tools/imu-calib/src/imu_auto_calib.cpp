#include<memory>
#include<cmath>
#include<vector>
#include<deque>
#include<iostream>

#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/imu.hpp"
#include"tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include"Eigen/Dense"
#include"Eigen/Geometry"


class ImuAutoCalibNode : public rclcpp::Node
{
public:
    ImuAutoCalibNode() : Node("imu_calib_node")
    {
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", 10,
        std::bind(&ImuAutoCalibNode::imuCallback, this, std::placeholders::_1)
    );
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/livox/imu_calib", 10
    );
    
    
    process_noise_cov = Eigen::Matrix3d::Identity()*(1e-6);
    measurement_noise_cov = Eigen::Matrix3d::Identity()*5.0;
    estimate_error_cov = Eigen::Matrix3d::Identity()*1.0;
    gravity_accel << 0.0, 0.0, 0.0;
    is_initialized_ = false;
    RCLCPP_INFO(this->get_logger(), "imu_calib_node卡尔曼滤波启动成功");
    };

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Vector3d z(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        if(!is_initialized_)
        {
            gravity_accel = z;
            is_initialized_ = true;
            return;
        }

        Eigen::Matrix3d P_pred = estimate_error_cov+process_noise_cov;
        Eigen::Matrix3d K = P_pred*(P_pred+measurement_noise_cov).inverse();
        gravity_accel = gravity_accel+K*(z-gravity_accel);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        estimate_error_cov = (I-K)*P_pred;

        static int stable_frames = 0;
        stable_frames++;

        if (stable_frames < 200) 
            return; 
        
        static int collect_count = 0;
        static Eigen::Vector3d sum_gravity = Eigen::Vector3d::Zero();
        
        if (collect_count < 500) 
        {
            sum_gravity += gravity_accel; 
            collect_count++;
            if (collect_count % 100 == 0) 
                RCLCPP_INFO(this->get_logger(), "采集中... %d/500", collect_count);
            return;
        } 
        else if (collect_count == 500) 
        {
            gravity_accel = sum_gravity / 500.0;
            collect_count++;
            RCLCPP_INFO(this->get_logger(), "采集完成，开始锁定最终值");
        }

        Eigen::Vector3d measured_gravity = gravity_accel.normalized();
        Eigen::Vector3d target_gravity(0.0,0.0,1.0);
        Eigen::Quaterniond q;
        q.setFromTwoVectors(measured_gravity, target_gravity);
        Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
        
        double roll = atan2(rotation_matrix(2,1), rotation_matrix(2,2));
        double pitch = atan2(-rotation_matrix(2,0),sqrt(pow(rotation_matrix(2,1),2)+pow(rotation_matrix(2,2),2)));
        double yaw = atan2(rotation_matrix(1,0), rotation_matrix(0,0));

        auto msg_calib = std::make_unique<sensor_msgs::msg::Imu>(*msg);
        msg_calib->linear_acceleration.x = gravity_accel(0);
        msg_calib->linear_acceleration.y = gravity_accel(1);
        msg_calib->linear_acceleration.z = gravity_accel(2);

        msg_calib->orientation.x = q.x();
        msg_calib->orientation.y = q.y();
        msg_calib->orientation.z = q.z();
        msg_calib->orientation.w = q.w();

        roll_deg = roll*180.0/M_PI;
        pitch_deg = pitch*180.0/M_PI;
        imu_publisher_->publish(std::move(msg_calib));
        
        static int count = 0;
        if(count++%50==0) {
            if (collect_count > 500) {
                RCLCPP_INFO(this->get_logger(),"【最终锁定】Roll: %.4f, Pitch: %.4f", roll_deg, pitch_deg);
            } else {
                RCLCPP_INFO(this->get_logger(),"第%d帧,卡尔曼滤波imu校准完成:roll:%.3f,pitch:%.3f",count, roll_deg,pitch_deg);
            }
        }
    };

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    Eigen::Vector3d gravity_accel;
    Eigen::Matrix3d estimate_error_cov;
    Eigen::Matrix3d process_noise_cov;
    Eigen::Matrix3d measurement_noise_cov;
    bool is_initialized_;
    double roll_deg;
    double pitch_deg;    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuAutoCalibNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}