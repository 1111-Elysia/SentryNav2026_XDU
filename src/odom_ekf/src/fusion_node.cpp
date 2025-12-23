#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <memory>

using std::placeholders::_1;

class FusionEKF : public rclcpp::Node {

public:
    FusionEKF()
    : Node("multi_odom_ekf_node")
    {
        declare_parameter("odom_topics", std::vector<std::string>());
        declare_parameter("publish_topic", "ekf_odom");
        declare_parameter("process_noise", 0.01);
        declare_parameter("meas_noise", 0.05);
        declare_parameter("chi2_threshold", 9.21);

        get_parameter("odom_topics", odom_topics_);
        get_parameter("publish_topic", output_topic_);
        get_parameter("process_noise", q_);
        get_parameter("meas_noise", r_);
        get_parameter("chi2_threshold", chi2_threshold_);

        // state dimension: x,y,theta
        x_.setZero(3,1);
        P_.setIdentity(3,3);

        for(auto &name : odom_topics_){
            auto callback = [this, name](nav_msgs::msg::Odometry::SharedPtr msg){
                this->odomCallback(msg);
            };
            subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                name, 10, callback));
        }

        pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

        last_time_ = now();
    }

private:

    //----------------------------
    // ODOMETRY INPUT HANDLING
    //----------------------------
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double dt = (now() - last_time_).seconds();
        if(dt <= 0.0) return;
        last_time_ = now();

        predict(dt);
        update(msg);
        publish();
    }

    //----------------------------
    // PREDICTION STAGE
    //----------------------------
    void predict(double dt)
    {
        // motion model = constant velocity (extend yourself)
        Eigen::Matrix<double,3,1> u;
        u << 0,0,0; 

        x_ = x_ + u * dt;

        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        P_ = F * P_ * F.transpose() + q_ * Eigen::Matrix3d::Identity();
    }

    //----------------------------
    // UPDATE STAGE
    //----------------------------
    void update(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Vector3d z;
        z << msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             0.0;

        Eigen::Vector3d z_pred = x_;

        Eigen::Vector3d r = z - z_pred;

        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S = H * P_ * H.transpose() + r_ * Eigen::Matrix3d::Identity();

        double chi2 = r.transpose() * S.inverse() * r;

        if(chi2 > chi2_threshold_)
        {
            RCLCPP_WARN(get_logger(),"measurement rejected");
            return;
        }

        // dynamic covariance scaling
        double scaling = 1.0 + (chi2 / 3.0);
        Eigen::Matrix3d R_scaled = scaling * r_ * Eigen::Matrix3d::Identity();

        Eigen::Matrix3d S_scaled = H * P_ * H.transpose() + R_scaled;

        Eigen::Matrix3d K = P_ * H.transpose() * S_scaled.inverse();

        x_ = x_ + K * r;
        P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
    }

    //----------------------------
    // PUBLISH RESULT
    //----------------------------
    void publish()
    {
        nav_msgs::msg::Odometry out;
        out.header.stamp = now();
        out.header.frame_id = "odom";
        out.child_frame_id = "ekf_livox_frame";

        out.pose.pose.position.x = x_(0);
        out.pose.pose.position.y = x_(1);

        pub_->publish(out);
    }

    //----------------------------
    // MEMBER VARIABLES
    //----------------------------
    std::vector<std::string> odom_topics_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

    std::string output_topic_;

    Eigen::Vector3d x_;
    Eigen::Matrix3d P_;

    double q_;
    double r_;
    double chi2_threshold_;

    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionEKF>());
    rclcpp::shutdown();
}
