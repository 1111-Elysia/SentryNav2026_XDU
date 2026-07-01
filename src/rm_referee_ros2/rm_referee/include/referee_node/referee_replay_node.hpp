#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "referee_node/referee_node.hpp"

class RefereeReplayNode : public RefereeNode {
 public:
  explicit RefereeReplayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                                                                      .allow_undeclared_parameters(true)
                                                                      .automatically_declare_parameters_from_overrides(true));
  ~RefereeReplayNode() override;

 private:
  void ReplayFile(const std::string& file_path, bool normal_link);
  bool WaitForTimestamp(uint64_t timestamp_us, uint64_t first_timestamp_us,
                        const std::chrono::steady_clock::time_point& replay_start);

  std::string normal_data_file_;
  std::string vt_data_file_;
  double replay_rate_{1.0};

  std::thread normal_replay_thread_;
  std::thread vt_replay_thread_;
  std::atomic<bool> stop_replay_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
};
