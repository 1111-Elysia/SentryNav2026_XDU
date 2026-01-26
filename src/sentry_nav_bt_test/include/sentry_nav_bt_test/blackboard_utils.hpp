#ifndef SENTRY_NAV_BT_TEST_BLACKBOARD_UTILS_HPP_
#define SENTRY_NAV_BT_TEST_BLACKBOARD_UTILS_HPP_

#include <rclcpp/logging.hpp>
#include <string>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace sentry_nav_bt_test
{
    namespace blackboard_utils
    {

        /**
         * @brief 从黑板获取特定类型的值并转换为double
         * @tparam T 要获取的数据类型
         * @param blackboard 黑板指针
         * @param key 黑板键名
         * @param result 转换结果
         * @return 是否成功获取并转换
         */
        template <typename T>
        bool getTypedValue(BT::Blackboard::Ptr blackboard, const std::string &key, double &result)
        {
            T value;
            if (blackboard->get(key, value))
            {
                if constexpr (std::is_same_v<T, bool>)
                {
                    result = value ? 1.0 : 0.0;
                }
                else
                {
                    result = static_cast<double>(value);
                }
                return true;
            }
            return false;
        }

        /**
         * @brief 从黑板获取值并转换为double (尝试多种类型)
         * @param blackboard 黑板指针
         * @param key 黑板键名
         * @param result 转换结果
         * @param logger_name 日志记录器名称，用于调试信息
         * @return 是否成功获取并转换
         */
        inline bool getValue(BT::Blackboard::Ptr blackboard, const std::string &key,
                             double &result, const std::string &logger_name = "BlackboardUtils")
        {
            // 尝试按指定顺序获取不同类型
            if (getTypedValue<int>(blackboard, key, result) ||
                getTypedValue<double>(blackboard, key, result) ||
                getTypedValue<float>(blackboard, key, result) ||
                getTypedValue<bool>(blackboard, key, result) ||
                getTypedValue<uint8_t>(blackboard, key, result) ||
                getTypedValue<uint16_t>(blackboard, key, result) ||
                getTypedValue<uint32_t>(blackboard, key, result))
            {
                return true;
            }

            RCLCPP_DEBUG(rclcpp::get_logger(logger_name),
                         "无法从黑板获取键 '%s' 的值，类型不支持或键不存在", key.c_str());
            return false;
        }

        /**
         * @brief 执行数值比较操作
         * @param value 第一个值
         * @param comparison 比较操作符
         * @param threshold 第二个值/阈值
         * @param logger_name 日志记录器名称，用于错误信息
         * @return 比较结果
         */
        inline bool compareValues(double value, const std::string &comparison,
                                  double threshold, const std::string &logger_name = "BlackboardUtils")
        {
            if (comparison == "gt" || comparison == ">")
            {
                return value > threshold;
            }
            else if (comparison == "lt" || comparison == "<")
            {
                return value < threshold;
            }
            else if (comparison == "eq" || comparison == "==")
            {
                return std::abs(value - threshold) < 1e-6; // 浮点数比较
            }
            else if (comparison == "gte" || comparison == ">=")
            {
                return value >= threshold;
            }
            else if (comparison == "lte" || comparison == "<=")
            {
                return value <= threshold;
            }
            else if (comparison == "neq" || comparison == "!=")
            {
                return std::abs(value - threshold) >= 1e-6;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name),
                             "无效的比较操作符 '%s'", comparison.c_str());
                return false;
            }
        }

        /**
         * @brief 尝试打印黑板中的值，支持多种类型
         * @param blackboard 黑板指针
         * @param key 黑板键名
         * @param prefix 打印前缀
         * @param logger 用于打印的日志对象
         * @return 是否成功打印
         */
        inline bool printValue(BT::Blackboard::Ptr blackboard, const std::string &key,
                               const std::string &prefix, rclcpp::Logger logger)
        {
            // 尝试打印基本类型
            try
            {
                // 尝试获取int
                int int_value;
                if (blackboard->get(key, int_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %d", prefix.c_str(), key.c_str(), int_value);
                    return true;
                }

                // 尝试获取double
                double double_value;
                if (blackboard->get(key, double_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %f", prefix.c_str(), key.c_str(), double_value);
                    return true;
                }

                // 尝试获取float
                float float_value;
                if (blackboard->get(key, float_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %f", prefix.c_str(), key.c_str(), static_cast<double>(float_value));
                    return true;
                }

                // 尝试获取bool
                bool bool_value;
                if (blackboard->get(key, bool_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %s", prefix.c_str(), key.c_str(), bool_value ? "true" : "false");
                    return true;
                }

                // 尝试获取string
                std::string string_value;
                if (blackboard->get(key, string_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %s", prefix.c_str(), key.c_str(), string_value.c_str());
                    return true;
                }

                // 尝试获取无符号整型
                uint8_t uint8_value;
                if (blackboard->get(key, uint8_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %d", prefix.c_str(), key.c_str(), static_cast<int>(uint8_value));
                    return true;
                }

                uint16_t uint16_value;
                if (blackboard->get(key, uint16_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %u", prefix.c_str(), key.c_str(), uint16_value);
                    return true;
                }

                uint32_t uint32_value;
                if (blackboard->get(key, uint32_value))
                {
                    RCLCPP_INFO(logger, "%s: %s = %u", prefix.c_str(), key.c_str(), uint32_value);
                    return true;
                }

                // 尝试获取PoseStamped
                try
                {
                    auto pose = blackboard->get<geometry_msgs::msg::PoseStamped>(key);
                    RCLCPP_INFO(logger,
                                "%s: %s = 位置(%.2f, %.2f, %.2f) 方向(%.2f, %.2f, %.2f, %.2f)",
                                prefix.c_str(), key.c_str(),
                                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                                pose.pose.orientation.x, pose.pose.orientation.y,
                                pose.pose.orientation.z, pose.pose.orientation.w);
                    return true;
                }
                catch (...)
                {
                    // 忽略处理
                }

                // 尝试获取向量类型
                try
                {
                    std::vector<int> int_vector;
                    if (blackboard->get(key, int_vector))
                    {
                        std::stringstream ss;
                        ss << "[";
                        for (size_t i = 0; i < int_vector.size(); ++i)
                        {
                            if (i > 0)
                                ss << ", ";
                            ss << int_vector[i];
                        }
                        ss << "]";
                        RCLCPP_INFO(logger, "%s: %s = %s", prefix.c_str(), key.c_str(), ss.str().c_str());
                        return true;
                    }
                }
                catch (...)
                {
                    // 忽略处理
                }

                try
                {
                    std::vector<double> double_vector;
                    if (blackboard->get(key, double_vector))
                    {
                        std::stringstream ss;
                        ss << "[";
                        for (size_t i = 0; i < double_vector.size(); ++i)
                        {
                            if (i > 0)
                                ss << ", ";
                            ss << double_vector[i];
                        }
                        ss << "]";
                        RCLCPP_INFO(logger, "%s: %s = %s", prefix.c_str(), key.c_str(), ss.str().c_str());
                        return true;
                    }
                }
                catch (...)
                {
                    // 忽略处理
                }

                try
                {
                    std::vector<uint8_t> uint8_vector;
                    if (blackboard->get(key, uint8_vector))
                    {
                        std::stringstream ss;
                        ss << "[总计" << uint8_vector.size() << "个元素, 前10个: ";
                        for (size_t i = 0; i < std::min(uint8_vector.size(), static_cast<size_t>(10)); ++i)
                        {
                            if (i > 0)
                                ss << ", ";
                            ss << static_cast<int>(uint8_vector[i]); // 转成int避免被解释为字符
                        }
                        if (uint8_vector.size() > 10)
                            ss << ", ...";
                        ss << "]";
                        RCLCPP_INFO(logger, "%s: %s = %s", prefix.c_str(), key.c_str(), ss.str().c_str());
                        return true;
                    }
                }
                catch (...)
                {
                    // 忽略处理
                }

                // 所有尝试都失败
                return false;
            }
            catch (const std::exception &e)
            {
                RCLCPP_DEBUG(logger, "尝试打印键 '%s' 时发生异常: %s", key.c_str(), e.what());
                return false;
            }
        }

        /**
         * @brief 设置黑板中的值，自动判断类型
         * @param blackboard 黑板指针
         * @param key 黑板键名
         * @param value_str 要设置的值的字符串表示
         * @param logger 用于日志记录的logger对象
         * @return 是否成功设置
         */
        inline bool setValue(BT::Blackboard::Ptr blackboard, const std::string &key,
                             const std::string &value_str, rclcpp::Logger logger)
        {
            // 尝试设置为布尔值
            if (value_str == "true" || value_str == "false")
            {
                bool value = (value_str == "true");
                blackboard->set(key, value);
                RCLCPP_DEBUG(logger, "设置黑板值 %s = %s (布尔型)", key.c_str(), value ? "true" : "false");
                return true;
            }

            // 尝试设置为整数
            try
            {
                int int_value = std::stoi(value_str);
                // 检查是否是纯整数
                if (value_str.find('.') == std::string::npos)
                {
                    blackboard->set(key, int_value);
                    RCLCPP_DEBUG(logger, "设置黑板值 %s = %d (整数型)", key.c_str(), int_value);
                    return true;
                }
            }
            catch (...)
            {
                // 转换失败，不是整数
            }

            // 尝试设置为浮点数
            try
            {
                double double_value = std::stod(value_str);
                blackboard->set(key, double_value);
                RCLCPP_DEBUG(logger, "设置黑板值 %s = %f (浮点型)", key.c_str(), double_value);
                return true;
            }
            catch (...)
            {
                // 转换失败，不是浮点数
            }

            // 作为字符串设置
            blackboard->set(key, value_str);
            RCLCPP_DEBUG(logger, "设置黑板值 %s = %s (字符串型)", key.c_str(), value_str.c_str());
            return true;
        }

    } // namespace blackboard_utils
} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_BLACKBOARD_UTILS_HPP_