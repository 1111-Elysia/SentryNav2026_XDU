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
 * @file  librm/modules/m3508_power_model.hpp
 * @brief M3508 电机功率模型
 * @ref   https://bbs.robomaster.com/article/9438
 * @ref   https://github.com/MaxwellDemonLin/Motor-modeling-and-power-control
 */

#ifndef LIBRM_MODULES_M3508_POWER_MODEL_HPP
#define LIBRM_MODULES_M3508_POWER_MODEL_HPP

#include <array>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief M3508 电机功率模型
 *
 * 基于西交利物浦大学RM2023-电机功率模型与功率控制开源实现
 * 功率模型公式：P = τ·ω/9. 55 + k1·ω² + k2·I² + C
 * 其中：τ 为输出转矩，ω 为转速(rpm)，I 为给定电流
 */
class M3508PowerModel {
 public:
  // 电机参数常量
  static constexpr f32 kKt = 0.3f / (3591.0f / 187.0f);                              ///< 转矩系数
  static constexpr f32 kKe = (60.0f / (2.0f * M_PI * 24.48f)) / (3591.0f / 187.0f);  ///< 反电动势系数
  static constexpr f32 kTorqueCoeff = 1.99688994e-6f;  ///< (20/16384)*(0.3)*(187/3591)/9.55

  // 拟合模型系数
  static constexpr f32 kK1 = 1.453e-07f;  ///< 转速平方项系数 (其他损耗-转速相关)
  static constexpr f32 kK2 = 1.23e-07f;   ///< 电流平方项系数 (其他损耗-电流相关)
  static constexpr f32 kConst = 4.081f;   ///< 常数项 (固定损耗)

  static constexpr f32 kCurrentToTorqueCurrent = 20.0f / 16384.0f;  ///< 控制电流到力矩电流转换
  static constexpr f32 kSpeedToMechanicalPower = 1.0f / 9.55f;      ///< rpm·N·m 到 W 的转换
  static constexpr i16 kMaxCurrent = 16000;                         ///< 最大控制电流限幅

  /**
   * @brief 电机状态结构体
   */
  struct MotorState {
    f32 speed_rpm;         ///< 当前转速 (rpm)
    f32 give_current;      ///< 当前给定电流 (控制值，范围 -16384~16384)
    f32 measured_current;  ///< 实际测量电流
  };

  /**
   * @brief 功率信息结构体
   */
  struct PowerInfo {
    f32 mechanical_power;  ///< 机械功率 (W)
    f32 loss_power;        ///< 损耗功率 (W)
    f32 total_power;       ///< 总功率 (W)
    f32 torque;            ///< 输出转矩 (N·m)
  };

  M3508PowerModel() = default;

  /**
   * @brief 计算电机当前功率
   * @param state 电机状态
   * @return 功率信息
   */
  PowerInfo CalculatePower(const MotorState& state) const {
    PowerInfo info;

    // 计算输出转矩:  τ = KT * I_torque
    f32 torque_current = state.give_current * kCurrentToTorqueCurrent;
    info.torque = kKt * torque_current;

    // 计算机械功率: P_mech = τ·ω / 9.55
    info.mechanical_power = (info.torque * state.speed_rpm) * kSpeedToMechanicalPower;

    // 计算损耗功率: P_loss = k1·ω² + k2·I² + C
    f32 speed_squared = state.speed_rpm * state.speed_rpm;
    f32 current_squared = state.give_current * state.give_current;
    info.loss_power = kK1 * speed_squared + kK2 * current_squared + kConst;

    // 总功率
    info.total_power = info.mechanical_power + info.loss_power;

    return info;
  }

  /**
   * @brief 根据目标功率计算所需的给定电流
   * @param target_power 目标功率 (W)
   * @param speed_rpm 当前转速 (rpm)
   * @param is_positive_direction 是否为正向加速
   * @return 计算得到的给定电流，如果无解则返回 0
   */
  f32 CalculateCtrlForPower(f32 target_power, f32 speed_rpm, bool is_positive_direction) const {
    // 求解二次方程: a·I² + b·I + c = 0
    // 其中功率模型为: P = (KT·I_torque)·ω/9.55 + k1·ω² + k2·I² + C
    //              P = TORQUE_COEFFICIENT·I·ω + k1·ω² + k2·I² + C

    f32 a = kK2;
    f32 b = kTorqueCoeff * speed_rpm;
    f32 c = kK1 * speed_rpm * speed_rpm - target_power + kConst;

    // 判别式
    f32 discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0) {
      // 无实数解，目标功率无法达到
      return 0.0f;
    }

    f32 sqrt_discriminant = std::sqrt(discriminant);
    f32 current;

    // 根据转矩方向选择解
    if (is_positive_direction) {
      current = (-b + sqrt_discriminant) / (2.0f * a);
    } else {
      current = (-b - sqrt_discriminant) / (2.0f * a);
    }

    // 限幅
    if (current > kMaxCurrent) {
      current = kMaxCurrent;
    } else if (current < -kMaxCurrent) {
      current = -kMaxCurrent;
    }

    return current;
  }

  /**
   * @brief       计算多电机功率分配（用于底盘功率控制）
   * @param[in]   motor_states 电机状态数组
   * @param[in]   initial_currents PID 计算的初始电流数组
   * @param[in]   max_total_power 最大总功率限制 (W)
   * @param[out]  output_currents 输出的限制后电流数组
   */
  template <usize NMotors>
  void DistributePower(const std::array<MotorState, NMotors> motor_states, const f32* initial_currents,
                       f32 max_total_power, f32* output_currents) const {
    // 计算初始总功率
    f32 initial_total_power = 0.0f;
    f32 initial_powers[NMotors];

    for (size_t i = 0; i < NMotors; ++i) {
      MotorState state = motor_states[i];
      state.give_current = initial_currents[i];

      const PowerInfo info = CalculatePower(state);
      initial_powers[i] = info.total_power;

      // 负功率不计入（过渡状态）
      if (initial_powers[i] > 0) {
        initial_total_power += initial_powers[i];
      }
    }

    // 如果超过最大功率，按比例缩放
    if (initial_total_power > max_total_power) {
      f32 power_scale = max_total_power / initial_total_power;

      for (size_t i = 0; i < NMotors; ++i) {
        f32 scaled_power = initial_powers[i] * power_scale;

        if (scaled_power < 0) {
          output_currents[i] = initial_currents[i];
          continue;
        }

        // 根据缩放后的功率重新计算电流
        bool is_positive = initial_currents[i] > 0;
        output_currents[i] = CalculateCtrlForPower(scaled_power, motor_states[i].speed_rpm, is_positive);
      }
    } else {
      // 不超过功率限制，直接使用初始电流
      for (size_t i = 0; i < NMotors; ++i) {
        output_currents[i] = initial_currents[i];
      }
    }
  }
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_M3508_POWER_MODEL_HPP