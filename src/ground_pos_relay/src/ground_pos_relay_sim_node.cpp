/**
 * @file ground_pos_relay_sim_node.cpp
 * @brief 监听 /ground_pos_sim/ground_robot_position（GUI 模拟器发布），
 *        将完整 40 字节通过 /rm_referee/tx 发送到 0x0301 / 0x0200 机器人间通信。
 *
 *        通过参数 robot_id 指定发送方 ID（默认 7）：
 *        - robot_id == 7   → sender=7,   receiver=9
 *        - robot_id == 107 → sender=107, receiver=109
 */

#include <rclcpp/rclcpp.hpp>
#include <rm_referee_msgs/msg/ground_robot_position.hpp>
#include <rm_referee_msgs/srv/tx.hpp>

#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

// ============================================================================
// 协议常量
// ============================================================================
constexpr uint8_t kSof = 0xA5;
constexpr uint8_t kHeaderLen = 5;
constexpr uint8_t kCmdIdLen = 2;
constexpr uint8_t kCrc16Len = 2;
constexpr uint8_t kAllMetadataLen = kHeaderLen + kCmdIdLen + kCrc16Len;  // 9

constexpr uint16_t kTargetCmdId = 0x0301;
constexpr uint16_t kSubContentId = 0x0200;
constexpr size_t kInteractionHeaderLen = 6;
constexpr size_t kGroundRobotPositionPayloadLen = 40;
constexpr size_t kInteractionDataLen = kInteractionHeaderLen + kGroundRobotPositionPayloadLen;

constexpr uint8_t kCrc8Init = 0xFF;
constexpr uint16_t kCrc16Init = 0xFFFF;

// ============================================================================
// CRC8 查表 (多项式: x^8 + x^5 + x^4 + 1)
// ============================================================================
static constexpr uint8_t kCrc8Table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

// ============================================================================
// CRC16 查表 (多项式: x^16 + x^12 + x^5 + 1, 即 CCITT/IBM)
// ============================================================================
static constexpr uint16_t kCrc16Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

// ============================================================================
// CRC 计算函数
// ============================================================================
static uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = kCrc8Init;
  while (len--) {
    crc = kCrc8Table[crc ^ *data++];
  }
  return crc;
}

static uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = kCrc16Init;
  while (len--) {
    crc = static_cast<uint16_t>((crc >> 8) ^ kCrc16Table[(crc ^ *data++) & 0xFF]);
  }
  return crc;
}

static void append_u16_le(std::vector<uint8_t> &buffer, uint16_t value) {
  buffer.push_back(static_cast<uint8_t>(value & 0xFF));
  buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

static void append_float32_le(std::vector<uint8_t> &buffer, float value) {
  static_assert(sizeof(float) == sizeof(uint32_t), "float32 must be 4 bytes");
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  buffer.push_back(static_cast<uint8_t>(bits & 0xFF));
  buffer.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
  buffer.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
  buffer.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));
}

// ============================================================================
// 主节点类
// ============================================================================
class GroundPosRelaySimNode : public rclcpp::Node {
 public:
  GroundPosRelaySimNode() : Node("ground_pos_relay_sim_node") {
    // 声明参数：robot_id，默认为 7
    this->declare_parameter<int>("robot_id", 7);

    robot_id_ = static_cast<uint8_t>(this->get_parameter("robot_id").as_int());

    if (robot_id_ != 7 && robot_id_ != 107) {
      RCLCPP_WARN(get_logger(),
                  "robot_id=%d is not 7 or 107; sender/receiver may be incorrect. "
                  "Set robot_id to 7 (sentry) or 107 (radar).",
                  static_cast<int>(robot_id_));
    }

    // 订阅 GUI 模拟器发布的己方地面机器人位置
    ground_pos_sub_ = create_subscription<rm_referee_msgs::msg::GroundRobotPosition>(
        "/ground_pos_sim/ground_robot_position",
        rclcpp::SensorDataQoS(),
        [this](const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg) {
          ground_pos_callback(msg);
        });

    // 创建 TX 服务客户端
    tx_client_ = create_client<rm_referee_msgs::srv::Tx>("/rm_referee/tx");

    RCLCPP_INFO(get_logger(),
                "GroundPosRelaySimNode started (robot_id=%d). "
                "Listening to /ground_pos_sim/ground_robot_position. "
                "Waiting for /rm_referee/tx service...",
                static_cast<int>(robot_id_));

    while (!tx_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO(get_logger(), "Service /rm_referee/tx is available.");
  }

 private:
  void ground_pos_callback(const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg) {
    // 确定 sender 和 receiver
    uint16_t sender_id = robot_id_;
    uint16_t receiver_id = (robot_id_ == 7) ? 9 : 109;

    // 0x020B 完整内容：10 个 float32，共 40 字节。
    const std::array<float, 10> payload_floats = {
        msg->hero_x, msg->hero_y,
        msg->engineer_x, msg->engineer_y,
        msg->standard_3_x, msg->standard_3_y,
        msg->standard_4_x, msg->standard_4_y,
        msg->reserved, msg->reserved_2,
    };

    for (const float value : payload_floats) {
      if (!std::isfinite(value)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "GroundRobotPosition contains non-finite value, skipping relay");
        return;
      }
    }

    // 构造数据段: data_cmd_id(2B) + sender_id(2B) + receiver_id(2B) + payload(40B) = 46B
    std::vector<uint8_t> data_segment;
    data_segment.reserve(kInteractionDataLen);

    append_u16_le(data_segment, kSubContentId);
    append_u16_le(data_segment, sender_id);
    append_u16_le(data_segment, receiver_id);

    for (const float value : payload_floats) {
      append_float32_le(data_segment, value);
    }

    // data_len = data_segment(46)
    uint16_t data_len = static_cast<uint16_t>(data_segment.size());

    // 构造完整帧: Header(5) + CmdID(2) + data_segment(46) + CRC16(2) = 55 bytes
    std::vector<uint8_t> frame;
    frame.reserve(kAllMetadataLen + data_len);

    // [0] SOF
    frame.push_back(kSof);
    // [1..2] DataLen
    append_u16_le(frame, data_len);
    // [3] Seq
    frame.push_back(seq_++);

    // [4] CRC8 (over bytes [0..3])
    frame.push_back(crc8(frame.data(), kHeaderLen - 1));

    // [5..6] CmdID (0x0301)
    append_u16_le(frame, kTargetCmdId);

    // [7..52] Data segment (46 bytes)
    frame.insert(frame.end(), data_segment.begin(), data_segment.end());

    // CRC16 over entire frame so far
    uint16_t crc16_val = crc16(frame.data(), frame.size());
    frame.push_back(crc16_val & 0xFF);
    frame.push_back((crc16_val >> 8) & 0xFF);

    // 通过 /rm_referee/tx 服务发送
    auto request = std::make_shared<rm_referee_msgs::srv::Tx::Request>();
    request->header.stamp = now();
    request->data = std::move(frame);

    const bool log_this_time = first_tx_log_.exchange(false);
    if (log_this_time) {
      RCLCPP_INFO(get_logger(), "start relaying simulated position data to 0x0301/0x0200...");
    }
    tx_client_->async_send_request(
        request,
        [this](rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedFuture future) {
          auto response = future.get();
          if (!response->ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "TX service reported failure");
          }
        });

    RCLCPP_DEBUG(get_logger(), "Relayed sim->0x0301/0x0200 (sender=%d, receiver=%d, seq=%d)",
                 sender_id, receiver_id, (seq_ - 1) & 0xFF);
  }

  // ---- 订阅 / 客户端 ----
  rclcpp::Subscription<rm_referee_msgs::msg::GroundRobotPosition>::SharedPtr ground_pos_sub_;
  rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedPtr tx_client_;

  // ---- 状态 ----
  uint8_t robot_id_{7};
  uint8_t seq_{0};
  std::atomic_bool first_tx_log_{true};
};

// ============================================================================
// main
// ============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundPosRelaySimNode>());
  rclcpp::shutdown();
  return 0;
}
