#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "serial_comm/serial_port.hpp"

class SerialCommNode : public rclcpp::Node
{
public:
  SerialCommNode() : Node("serial_comm_node")
  {
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    
    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    
    if (!serial_.open(port, baudrate)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
      return;
    }
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SerialCommNode::readLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Serial communication node initialized");
  }

  ~SerialCommNode()
  {
    serial_.close();
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    uint8_t buffer[16];
    // Pack cmd_vel data into buffer
    memcpy(&buffer[0], &msg->linear.x, sizeof(float));
    memcpy(&buffer[4], &msg->linear.y, sizeof(float));
    memcpy(&buffer[8], &msg->angular.z, sizeof(float));
    
    serial_.write(buffer, 12);
  }

  void readLoop()
  {
    uint8_t buffer[256];
    int bytes = serial_.read(buffer, sizeof(buffer));
    if (bytes > 0) {
      // Process received data
    }
  }

  SerialPort serial_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialCommNode>());
  rclcpp::shutdown();
  return 0;
}
