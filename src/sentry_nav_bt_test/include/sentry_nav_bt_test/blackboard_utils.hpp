#ifndef SENTRY_NAV_BT_TEST_BLACKBOARD_UTILS_HPP_
#define SENTRY_NAV_BT_TEST_BLACKBOARD_UTILS_HPP_

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
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
         * @brief 将黑板中的值格式化为字符串，支持多种类型
         * @param blackboard 黑板指针
         * @param key 黑板键名
         * @param formatted_value 输出的格式化字符串
         * @return 是否成功格式化
         */
        inline bool formatValue(BT::Blackboard::Ptr blackboard,
                                const std::string &key,
                                std::string &formatted_value)
        {
            try
            {
                int int_value;
                if (blackboard->get(key, int_value))
                {
                    formatted_value = std::to_string(int_value);
                    return true;
                }

                double double_value;
                if (blackboard->get(key, double_value))
                {
                    std::ostringstream oss;
                    oss << double_value;
                    formatted_value = oss.str();
                    return true;
                }

                float float_value;
                if (blackboard->get(key, float_value))
                {
                    std::ostringstream oss;
                    oss << static_cast<double>(float_value);
                    formatted_value = oss.str();
                    return true;
                }

                bool bool_value;
                if (blackboard->get(key, bool_value))
                {
                    formatted_value = bool_value ? "true" : "false";
                    return true;
                }

                std::string string_value;
                if (blackboard->get(key, string_value))
                {
                    formatted_value = string_value;
                    return true;
                }

                uint8_t uint8_value;
                if (blackboard->get(key, uint8_value))
                {
                    formatted_value = std::to_string(static_cast<int>(uint8_value));
                    return true;
                }

                uint16_t uint16_value;
                if (blackboard->get(key, uint16_value))
                {
                    formatted_value = std::to_string(uint16_value);
                    return true;
                }

                uint32_t uint32_value;
                if (blackboard->get(key, uint32_value))
                {
                    formatted_value = std::to_string(uint32_value);
                    return true;
                }

                try
                {
                    auto pose = blackboard->get<geometry_msgs::msg::PoseStamped>(key);
                    std::ostringstream oss;
                    oss << "位置("
                        << pose.pose.position.x << ", "
                        << pose.pose.position.y << ", "
                        << pose.pose.position.z << ") 方向("
                        << pose.pose.orientation.x << ", "
                        << pose.pose.orientation.y << ", "
                        << pose.pose.orientation.z << ", "
                        << pose.pose.orientation.w << ")";
                    formatted_value = oss.str();
                    return true;
                }
                catch (...)
                {
                }

                try
                {
                    std::vector<int> int_vector;
                    if (blackboard->get(key, int_vector))
                    {
                        std::ostringstream oss;
                        oss << "[";
                        for (size_t i = 0; i < int_vector.size(); ++i)
                        {
                            if (i > 0) {
                                oss << ", ";
                            }
                            oss << int_vector[i];
                        }
                        oss << "]";
                        formatted_value = oss.str();
                        return true;
                    }
                }
                catch (...)
                {
                }

                try
                {
                    std::vector<double> double_vector;
                    if (blackboard->get(key, double_vector))
                    {
                        std::ostringstream oss;
                        oss << "[";
                        for (size_t i = 0; i < double_vector.size(); ++i)
                        {
                            if (i > 0) {
                                oss << ", ";
                            }
                            oss << double_vector[i];
                        }
                        oss << "]";
                        formatted_value = oss.str();
                        return true;
                    }
                }
                catch (...)
                {
                }

                try
                {
                    std::vector<uint8_t> uint8_vector;
                    if (blackboard->get(key, uint8_vector))
                    {
                        std::ostringstream oss;
                        oss << "[总计" << uint8_vector.size() << "个元素, 前10个: ";
                        for (size_t i = 0; i < std::min(uint8_vector.size(), static_cast<size_t>(10)); ++i)
                        {
                            if (i > 0) {
                                oss << ", ";
                            }
                            oss << static_cast<int>(uint8_vector[i]);
                        }
                        if (uint8_vector.size() > 10) {
                            oss << ", ...";
                        }
                        oss << "]";
                        formatted_value = oss.str();
                        return true;
                    }
                }
                catch (...)
                {
                }

                return false;
            }
            catch (...)
            {
                return false;
            }
        }

        inline std::string formatKeyValueMessage(const std::string &key,
                                                 const std::string &prefix,
                                                 const std::string &formatted_value)
        {
            if (prefix.empty())
            {
                return key + " = " + formatted_value;
            }

            return prefix + ": " + key + " = " + formatted_value;
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
                std::string formatted_value;
                if (!formatValue(blackboard, key, formatted_value))
                {
                    return false;
                }

                const auto log_message = formatKeyValueMessage(key, prefix, formatted_value);
                RCLCPP_INFO(logger, "%s", log_message.c_str());
                return true;
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
