/*
  Copyright (c) 2025 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @file  librm/device/device.hpp
 * @brief 驱动框架的设备基类，主要功能为监视设备在线状态
 */

#ifndef LIBRM_DEVICE_DEVICE_HPP
#define LIBRM_DEVICE_DEVICE_HPP

#include <chrono>
#include <functional>
#include <initializer_list>

#include <etl/vector.h>
#include <etl/delegate.h>
#include <etl/string.h>

#include "librm/core/exception.hpp"
#include "librm/core/typedefs.hpp"

namespace rm::device {

/**
 * @brief 驱动框架的设备基类，功能为监视设备状态
 */
class Device {
  using time_point = std::chrono::time_point<std::chrono::steady_clock>;
  using duration = std::chrono::steady_clock::duration;

  constexpr static usize kMaxNameLength = 32;

 public:
  Device();
  virtual ~Device() = default;

  /**
   * @brief 设置心跳超时时间
   * @param timeout 心跳超时时间
   * @note  如果超过这个时间没有收到心跳则认为设备离线，默认值为1秒
   */
  void SetHeartbeatTimeout(duration timeout);

  /**
   * @brief 设备状态
   */
  enum Status {
    kUnknown = -1,  ///< 未知
    kOffline,       ///< 离线
    kFault,         ///< 在线但存在故障（比如电机过流）
    kOk,            ///< 在线且正常工作
  };

  /**
   * @return 设备的当前状态
   */
  [[nodiscard]] Status online_status();

  /**
   * @return 获取最后一次设备上报状态的时间点
   */
  [[nodiscard]] time_point last_seen() const;

  /**
   * @brief 设置设备名称
   * @param name 设备名称
   */
  void SetName(etl::string<kMaxNameLength> name);

  /**
   * @brief 获取设备名称
   * @return 设备名称
   */
  [[nodiscard]] etl::string<kMaxNameLength> name() const;

 protected:
  /**
   * @brief 更新设备状态
   * @note  子类应该在确定设备仍然在线（比如收到反馈报文）时调用此函数，更新设备在线时间戳
   */
  void ReportStatus(Status status);

 private:
  etl::string<kMaxNameLength> name_{};       ///< 设备名称，最大32字节
  Status online_status_{kUnknown};           ///< 设备当前在线状态
  time_point last_seen_{time_point::min()};  ///< 设备最后一次上报状态的时间点
  duration heartbeat_timeout_{std::chrono::seconds(1)};  ///< 心跳超时时间，超过这个时间没有收到心跳则认为设备离线
};

/**
 * @brief 设备管理器，用来维护多个设备的在线状态
 * @tparam kMaxDevices 最大容纳的设备数量，按需设置
 * @tparam kUseStdFunctionCallback 是否使用 std::function 作为回调类型，默认为 true。如果设置为 false 则使用
 * etl::delegate（类似C++26 std::function_ref），无动态内存分配但使用起来不如 std::function 方便
 */
template <size_t kMaxDevices, bool kUseStdFunctionCallback = true>
class DeviceManager {
  using CallbackType =                                   //
      std::conditional_t<kUseStdFunctionCallback,        //
                         std::function<void(Device *)>,  //
                         etl::delegate<void(Device *)>>;

 public:
  DeviceManager() = default;

  DeviceManager(std::initializer_list<Device *> devices) {
    if (devices.size() <= kMaxDevices) {
      for (auto *device : devices) {
        devices_.push_back({device, Device::kUnknown});  // 初始状态为未知
      }
    } else {
      Throw(std::runtime_error("DeviceManager: Too many devices!"));
    }
  }

  DeviceManager &operator<<(Device *device) {
    if (devices_.size() < kMaxDevices) {
      devices_.push_back({device, Device::kUnknown});  // 初始状态为未知
    } else {
      Throw(std::runtime_error("DeviceManager: Too many devices!"));
    }
    return *this;
  }

  /**
   * @brief 更新所有设备的在线状态
   * @return 所有设备是否在线且正常工作
   */
  bool Update() {
    all_device_ok_ = true;
    for (auto &entry : devices_) {
      const Device::Status current_status = entry.device->online_status();
      if (current_status != Device::kOk) {
        all_device_ok_ = false;
      }
      // 设备状态发生变化，触发回调
      if (entry.prev_status != current_status) {
        for (const auto &callback : status_change_callbacks_) {
          callback(entry.device);
        }
      }
      entry.prev_status = current_status;
    }
    return all_device_ok_;
  }

  /**
   * @brief 注册设备状态变化回调
   * @param callback 回调函数，参数为状态发生变化的设备指针
   * @note  当设备的状态发生任何变化时都会触发此回调，包括：在线<->离线、在线<->故障、离线<->故障等所有状态转换
   */
  void OnDeviceStatusChange(CallbackType callback) {
    if (status_change_callbacks_.size() < 10) {
      status_change_callbacks_.emplace_back(callback);
    } else {
      Throw(std::runtime_error("DeviceManager: Too many status change callbacks!"));
    }
  }

  /**
   * @return 所有设备是否在线且正常工作
   */
  bool all_device_ok() const { return all_device_ok_; }

 private:
  /**
   * @brief 设备条目，封装设备指针和其上一次的状态
   */
  struct DeviceEntry {
    Device *device{nullptr};                       ///< 设备指针
    Device::Status prev_status{Device::kUnknown};  ///< 上一次的设备状态
  };

  constexpr static usize kMaxCallbacks = 4;  ///< 最大回调函数数量，不够可以调大

  bool all_device_ok_{false};
  etl::vector<CallbackType, kMaxCallbacks> status_change_callbacks_;
  etl::vector<DeviceEntry, kMaxDevices> devices_;
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_DEVICE_HPP
