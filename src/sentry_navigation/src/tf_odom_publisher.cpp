// ...existing code...

private:
    void timerCallback()
    {
        // 尝试获取最新 TF，失败则回退到缓存
        geometry_msgs::msg::TransformStamped map_to_baselink_msg;
        geometry_msgs::msg::TransformStamped odom_to_baselink_msg;
        rclcpp::Time now = this->now();

        // 获取 map->base_link
        bool got_map_bl = false;
        try {
            map_to_baselink_msg = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
            last_map_to_baselink_msg_ = map_to_baselink_msg;
            have_map_tf_ = true;
            got_map_bl = true;
        } catch (const tf2::TransformException &ex) {
            if (have_map_tf_) {
                // 回退：使用最近一次成功的 TF（允许很短的回退窗口）
                const double age = (now - last_map_to_baselink_msg_.header.stamp).seconds();
                if (age < 0.3) {
                    map_to_baselink_msg = last_map_to_baselink_msg_;
                    got_map_bl = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "map->base_link 不可用且缓存过旧(%.2fs): %s", age, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 map->base_link: %s", ex.what());
            }
        }

        // 获取 odom->base_link
        bool got_odom_bl = false;
        try {
            odom_to_baselink_msg = tf_buffer_.lookupTransform("odom", "base_link", rclcpp::Time(0));
            last_odom_to_baselink_msg_ = odom_to_baselink_msg;
            have_odom_tf_ = true;
            got_odom_bl = true;
        } catch (const tf2::TransformException &ex) {
            if (have_odom_tf_) {
                const double age = (now - last_odom_to_baselink_msg_.header.stamp).seconds();
                if (age < 0.5) { // odom只有10Hz，放宽容忍
                    odom_to_baselink_msg = last_odom_to_baselink_msg_;
                    got_odom_bl = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "odom->base_link 不可用且缓存过旧(%.2fs): %s", age, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 odom->base_link: %s", ex.what());
            }
        }

        if (got_map_bl && got_odom_bl)
        {
            try {
                // 计算并发布 map->odom
                tf2::Transform tf_map_to_baselink, tf_odom_to_baselink;
                tf2::fromMsg(map_to_baselink_msg.transform, tf_map_to_baselink);
                tf2::fromMsg(odom_to_baselink_msg.transform, tf_odom_to_baselink);

                tf2::Transform tf_map_to_odom = tf_map_to_baselink * tf_odom_to_baselink.inverse();

                geometry_msgs::msg::TransformStamped map_to_odom_msg;
                // 用 map->base_link 的时间戳（高频、稳定），若是回退则用 now
                const bool used_cached_map = (map_to_baselink_msg.header.stamp == last_map_to_baselink_msg_.header.stamp) && !got_map_bl;
                map_to_odom_msg.header.stamp = got_map_bl ? map_to_baselink_msg.header.stamp : now;
                map_to_odom_msg.header.frame_id = "map";
                map_to_odom_msg.child_frame_id = "odom";
                map_to_odom_msg.transform = tf2::toMsg(tf_map_to_odom);

                tf_broadcaster_->sendTransform(map_to_odom_msg);

                if (!has_published_tf_) {
                    RCLCPP_INFO(this->get_logger(), "成功发布 map->odom TF");
                    has_published_tf_ = true;
                }
                has_warned_tf_ = false;
            } catch (const std::exception &e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "计算/发布 map->odom 失败: %s", e.what());
            }
        }

        // 发布 /odom（保持不变）
        if (!has_fastlio_) return;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = last_fastlio_stamp_;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose = vehiclePoseToROS(odom_pose_vehicle_);

        double vx, vy, vz, wx, wy, wz;
        vehicleVelToROS(vel_vx_, vel_vy_, vel_vz_, vel_wx_, vel_wy_, vel_wz_, vx, vy, vz, wx, wy, wz);
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.linear.z = vz;
        odom_msg.twist.twist.angular.x = wx;
        odom_msg.twist.twist.angular.y = wy;
        odom_msg.twist.twist.angular.z = wz;

        odom_msg.pose.covariance[0] = 0.01;
        odom_msg.pose.covariance[7] = 0.01;
        odom_msg.pose.covariance[35] = 0.01;
        odom_msg.twist.covariance[0] = 0.05;
        odom_msg.twist.covariance[7] = 0.05;
        odom_msg.twist.covariance[35] = 0.05;

        odom_publisher_->publish(odom_msg);
    }

    // ...existing code...

private:
    // ...existing members...
    geometry_msgs::msg::TransformStamped last_map_to_baselink_msg_;
    geometry_msgs::msg::TransformStamped last_odom_to_baselink_msg_;
    bool have_map_tf_ = false;
    bool have_odom_tf_ = false;

// ...existing code...