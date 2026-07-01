#include "referee_node/referee_replay_node.hpp"

#include <cmath>
#include <fstream>

namespace {
constexpr uint32_t kMaxRecordSize = 16U * 1024U * 1024U;
}

RefereeReplayNode::RefereeReplayNode(const rclcpp::NodeOptions& options)
    : RefereeNode("referee_replay_node", options, false) {
  normal_data_file_ = declare_parameter("normal_data_file", "");
  vt_data_file_ = declare_parameter("vt_data_file", "");
  replay_rate_ = declare_parameter("replay_rate", 1.0);

  if (replay_rate_ < 0.0 || !std::isfinite(replay_rate_)) {
    RCLCPP_WARN(get_logger(), "Invalid replay_rate %.3f, using 1.0", replay_rate_);
    replay_rate_ = 1.0;
  }

  if (normal_data_file_.empty() && vt_data_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "No replay file configured; set normal_data_file or vt_data_file");
    return;
  }

  if (!normal_data_file_.empty()) {
    normal_replay_thread_ = std::thread([this] { ReplayFile(normal_data_file_, true); });
  }
  if (!vt_data_file_.empty()) {
    vt_replay_thread_ = std::thread([this] { ReplayFile(vt_data_file_, false); });
  }
}

RefereeReplayNode::~RefereeReplayNode() {
  stop_replay_ = true;
  wait_condition_.notify_all();

  if (normal_replay_thread_.joinable()) {
    normal_replay_thread_.join();
  }
  if (vt_replay_thread_.joinable()) {
    vt_replay_thread_.join();
  }
}

void RefereeReplayNode::ReplayFile(const std::string& file_path, bool normal_link) {
  const char* link_name = normal_link ? "normal" : "VT";
  std::ifstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s replay file: %s", link_name, file_path.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Replaying %s raw data from: %s (rate=%.3f)", link_name, file_path.c_str(),
              replay_rate_);

  bool first_record = true;
  uint64_t first_timestamp_us = 0;
  auto replay_start = std::chrono::steady_clock::now();
  size_t record_count = 0;

  while (!stop_replay_) {
    uint64_t timestamp_us = 0;
    uint32_t data_size = 0;

    file.read(reinterpret_cast<char*>(&timestamp_us), sizeof(timestamp_us));
    if (file.eof() && file.gcount() == 0) {
      break;
    }
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Truncated timestamp in %s replay file after %zu records", link_name, record_count);
      return;
    }

    file.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Truncated data length in %s replay file after %zu records", link_name, record_count);
      return;
    }
    if (data_size > kMaxRecordSize) {
      RCLCPP_ERROR(get_logger(), "Invalid %s replay record size: %u bytes", link_name, data_size);
      return;
    }

    std::string data(data_size, '\0');
    file.read(data.data(), static_cast<std::streamsize>(data.size()));
    if (!file) {
      RCLCPP_ERROR(get_logger(), "Truncated payload in %s replay file after %zu records", link_name, record_count);
      return;
    }

    if (first_record) {
      first_timestamp_us = timestamp_us;
      replay_start = std::chrono::steady_clock::now();
      first_record = false;
    } else if (timestamp_us < first_timestamp_us) {
      RCLCPP_ERROR(get_logger(), "Non-monotonic timestamp in %s replay file after %zu records", link_name,
                   record_count);
      return;
    }

    if (!WaitForTimestamp(timestamp_us, first_timestamp_us, replay_start)) {
      return;
    }

    if (normal_link) {
      FeedNormalData(data);
    } else {
      FeedVtData(data);
    }
    ++record_count;
  }

  if (!stop_replay_) {
    RCLCPP_INFO(get_logger(), "%s raw data replay completed: %zu records", link_name, record_count);
  }
}

bool RefereeReplayNode::WaitForTimestamp(uint64_t timestamp_us, uint64_t first_timestamp_us,
                                         const std::chrono::steady_clock::time_point& replay_start) {
  if (replay_rate_ == 0.0) {
    return !stop_replay_;
  }

  const auto elapsed_us = timestamp_us - first_timestamp_us;
  const auto scaled_us = static_cast<uint64_t>(static_cast<double>(elapsed_us) / replay_rate_);
  const auto target = replay_start + std::chrono::microseconds(scaled_us);
  std::unique_lock<std::mutex> lock(wait_mutex_);
  return !wait_condition_.wait_until(lock, target, [this] { return stop_replay_.load(); });
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RefereeReplayNode)
