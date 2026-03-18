#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>

class SuperDeluo
{
public:
	struct Output
	{
		bool active{false};
		float vx{0.0f};
		float vy{0.0f};
		float vw{0.0f};
	};

	explicit SuperDeluo(rclcpp::Node & node)
	: node_(node),
	  tf_buffer_(node.get_clock()),
	  tf_listener_(tf_buffer_),
	  rng_(std::random_device{}())
	{
		node_.declare_parameter<double>("super_deluo.duration_s", 5.0);
		node_.declare_parameter<double>("super_deluo.vw_min", 0.9);
		node_.declare_parameter<double>("super_deluo.vw_max", 1.0);
		node_.declare_parameter<double>("super_deluo.circle_radius_m", 0.35);
		node_.declare_parameter<double>("super_deluo.circle_tangent_speed", 0.25);
		node_.declare_parameter<double>("super_deluo.radial_kp", 1.1);
		node_.declare_parameter<double>("super_deluo.max_vxy", 0.40);
		node_.declare_parameter<double>("super_deluo.vxy_cmd_vel_low_prio_eps", 0.03);
		node_.declare_parameter<std::string>("super_deluo.map_frame", "map");
		node_.declare_parameter<std::string>("super_deluo.base_frame", "base_link");

		duration_s_ = node_.get_parameter("super_deluo.duration_s").as_double();
		vw_min_ = node_.get_parameter("super_deluo.vw_min").as_double();
		vw_max_ = node_.get_parameter("super_deluo.vw_max").as_double();
		circle_radius_m_ = node_.get_parameter("super_deluo.circle_radius_m").as_double();
		circle_tangent_speed_ = node_.get_parameter("super_deluo.circle_tangent_speed").as_double();
		radial_kp_ = node_.get_parameter("super_deluo.radial_kp").as_double();
		max_vxy_ = node_.get_parameter("super_deluo.max_vxy").as_double();
		vxy_cmd_vel_low_prio_eps_ = node_.get_parameter("super_deluo.vxy_cmd_vel_low_prio_eps").as_double();
		map_frame_ = node_.get_parameter("super_deluo.map_frame").as_string();
		base_frame_ = node_.get_parameter("super_deluo.base_frame").as_string();

		if (vw_min_ > vw_max_) {
			std::swap(vw_min_, vw_max_);
		}
		vw_dis_ = std::uniform_real_distribution<double>(vw_min_, vw_max_);
	}

	void onHurt(const rclcpp::Time & now)
	{
		hurt_active_ = true;
		hurt_start_time_ = now;
		last_vw_switch_time_ = now;
		current_vw_ = static_cast<float>(vw_dis_(rng_));

		double x = 0.0;
		double y = 0.0;
		if (getCurrentPoseXY(x, y)) {
			circle_center_x_ = x;
			circle_center_y_ = y;
			have_center_ = true;
		} else {
			have_center_ = false;
		}

		RCLCPP_WARN(
			node_.get_logger(),
			"super_deluo trigger: random vw [%.2f, %.2f], duration %.2fs",
			vw_min_, vw_max_, duration_s_);
	}

	Output compute(const rclcpp::Time & now)
	{
		Output out{};
		if (!hurt_active_) {
			return out;
		}

		const double dt_total = (now - hurt_start_time_).seconds();
		if (dt_total >= duration_s_) {
			hurt_active_ = false;
			current_vw_ = 0.0f;
			return out;
		}

		if ((now - last_vw_switch_time_).seconds() >= 1.0) {
			current_vw_ = static_cast<float>(vw_dis_(rng_));
			last_vw_switch_time_ = now;
		}

		out.active = true;
		out.vw = current_vw_;
		computeCircleVxy(out.vx, out.vy);
		return out;
	}

	bool shouldInjectVxy(float cmd_vx, float cmd_vy) const
	{
		return std::fabs(static_cast<double>(cmd_vx)) < vxy_cmd_vel_low_prio_eps_ &&
			   std::fabs(static_cast<double>(cmd_vy)) < vxy_cmd_vel_low_prio_eps_;
	}

private:
	bool getCurrentPoseXY(double & x, double & y)
	{
		try {
			auto tf = tf_buffer_.lookupTransform(
				map_frame_,
				base_frame_,
				tf2::TimePointZero,
				tf2::durationFromSec(0.02));
			x = tf.transform.translation.x;
			y = tf.transform.translation.y;
			return true;
		} catch (const tf2::TransformException & ex) {
			(void)ex;
			return false;
		}
	}

	void computeCircleVxy(float & vx, float & vy)
	{
		double x = 0.0;
		double y = 0.0;
		if (!getCurrentPoseXY(x, y)) {
			vx = 0.0f;
			vy = 0.0f;
			return;
		}

		if (!have_center_) {
			circle_center_x_ = x;
			circle_center_y_ = y;
			have_center_ = true;
		}

		const double dx = x - circle_center_x_;
		const double dy = y - circle_center_y_;
		const double r = std::hypot(dx, dy);

		double ux = 1.0;
		double uy = 0.0;
		if (r > 1e-3) {
			ux = dx / r;
			uy = dy / r;
		}

		const double tx = -uy;
		const double ty = ux;
		const double radial_error = circle_radius_m_ - r;

		double vx_cmd = circle_tangent_speed_ * tx + radial_kp_ * radial_error * ux;
		double vy_cmd = circle_tangent_speed_ * ty + radial_kp_ * radial_error * uy;

		vx_cmd = std::clamp(vx_cmd, -max_vxy_, max_vxy_);
		vy_cmd = std::clamp(vy_cmd, -max_vxy_, max_vxy_);

		vx = static_cast<float>(vx_cmd);
		vy = static_cast<float>(vy_cmd);
	}

private:
	rclcpp::Node & node_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	bool hurt_active_{false};
	rclcpp::Time hurt_start_time_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_vw_switch_time_{0, 0, RCL_ROS_TIME};

	double duration_s_{5.0};
	double vw_min_{0.9};
	double vw_max_{1.0};
	double circle_radius_m_{0.35};
	double circle_tangent_speed_{0.25};
	double radial_kp_{1.1};
	double max_vxy_{0.40};
	double vxy_cmd_vel_low_prio_eps_{0.03};
	std::string map_frame_{"map"};
	std::string base_frame_{"base_link"};

	bool have_center_{false};
	double circle_center_x_{0.0};
	double circle_center_y_{0.0};

	std::mt19937 rng_;
	std::uniform_real_distribution<double> vw_dis_{0.9, 1.0};
	float current_vw_{0.0f};
};

