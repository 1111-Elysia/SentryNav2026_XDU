#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <random>

class SuperDeluo
{
public:
	struct Output
	{
		bool active{false};
		float vw{0.0f};
	};

	explicit SuperDeluo(rclcpp::Node & node)
	: node_(node),
	  rng_(std::random_device{}())
	{
		node_.declare_parameter<double>("super_deluo.duration_s", 5.0);
		node_.declare_parameter<double>("super_deluo.vw_min", 0.9);
		node_.declare_parameter<double>("super_deluo.vw_max", 1.0);

		duration_s_ = node_.get_parameter("super_deluo.duration_s").as_double();
		vw_min_ = node_.get_parameter("super_deluo.vw_min").as_double();
		vw_max_ = node_.get_parameter("super_deluo.vw_max").as_double();

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
		return out;
	}

private:
	rclcpp::Node & node_;

	bool hurt_active_{false};
	rclcpp::Time hurt_start_time_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_vw_switch_time_{0, 0, RCL_ROS_TIME};

	double duration_s_{5.0};
	double vw_min_{0.9};
	double vw_max_{1.0};

	std::mt19937 rng_;
	std::uniform_real_distribution<double> vw_dis_{0.9, 1.0};
	float current_vw_{0.0f};
};

