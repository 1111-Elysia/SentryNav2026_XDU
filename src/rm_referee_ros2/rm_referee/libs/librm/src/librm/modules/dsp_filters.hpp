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
 * @file  librm/modules/dsp_filters.hpp
 * @brief 一些常用的信号处理滤波器，来自Ardupilot的Filter模块，本文件对其进行了简单的封装和类型别名定义。
 *
 * 使用示例：
 * @code
 *   rm::modules::LowPassFilter<float> lpf;
 *   lpf.set_cutoff_frequency(10.0f);
 *   float filtered = lpf.apply(raw_value, dt);
 * @endcode
 */

#ifndef LIBRM_MODULES_DSP_FILTERS_HPP
#define LIBRM_MODULES_DSP_FILTERS_HPP

#include <ap_filters/Filter.h>
#include <ap_filters/FilterClass.h>
#include <ap_filters/FilterWithBuffer.h>
#include <ap_filters/AverageFilter.h>
#include <ap_filters/LowPassFilter.h>
#include <ap_filters/LowPassFilter2p.h>
#include <ap_filters/NotchFilter.h>
#include <ap_filters/HarmonicNotchFilter.h>
#include <ap_filters/DerivativeFilter.h>
#include <ap_filters/ModeFilter.h>
#include <ap_filters/SlewLimiter.h>
#include <ap_filters/Butter.h>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

// ============================================================================
// 基础滤波器类型别名
// ============================================================================

/// 滤波器基类
template <typename T>
using Filter = ::Filter<T>;

/// 带缓冲区的滤波器基类
template <typename T, uint8_t FILTER_SIZE>
using FilterWithBuffer = ::FilterWithBuffer<T, FILTER_SIZE>;

// ============================================================================
// 平均滤波器
// ============================================================================

/// 平均滤波器模板类
/// @tparam T 数据类型
/// @tparam U 求和时使用的更大数据类型，用于防止溢出
/// @tparam FILTER_SIZE 滤波器缓冲区大小
template <typename T, typename U, uint8_t FILTER_SIZE>
using AverageFilter = ::AverageFilter<T, U, FILTER_SIZE>;

/// 积分平均滤波器
template <typename T, typename U, uint8_t FILTER_SIZE>
using AverageIntegralFilter = ::AverageIntegralFilter<T, U, FILTER_SIZE>;

// 常用平均滤波器类型别名
using AverageFilterInt8_Size2 = ::AverageFilterInt8_Size2;
using AverageFilterInt8_Size3 = ::AverageFilterInt8_Size3;
using AverageFilterInt8_Size4 = ::AverageFilterInt8_Size4;
using AverageFilterInt8_Size5 = ::AverageFilterInt8_Size5;
using AverageFilterUInt8_Size2 = ::AverageFilterUInt8_Size2;
using AverageFilterUInt8_Size3 = ::AverageFilterUInt8_Size3;
using AverageFilterUInt8_Size4 = ::AverageFilterUInt8_Size4;
using AverageFilterUInt8_Size5 = ::AverageFilterUInt8_Size5;
using AverageFilterUInt16_Size2 = ::AverageFilterUInt16_Size2;
using AverageFilterUInt16_Size3 = ::AverageFilterUInt16_Size3;
using AverageFilterUInt16_Size4 = ::AverageFilterUInt16_Size4;
using AverageFilterUInt16_Size5 = ::AverageFilterUInt16_Size5;
using AverageFilterInt16_Size2 = ::AverageFilterInt16_Size2;
using AverageFilterInt16_Size3 = ::AverageFilterInt16_Size3;
using AverageFilterInt16_Size4 = ::AverageFilterInt16_Size4;
using AverageFilterInt16_Size5 = ::AverageFilterInt16_Size5;
using AverageFilterInt32_Size2 = ::AverageFilterInt32_Size2;
using AverageFilterInt32_Size3 = ::AverageFilterInt32_Size3;
using AverageFilterInt32_Size4 = ::AverageFilterInt32_Size4;
using AverageFilterInt32_Size5 = ::AverageFilterInt32_Size5;
using AverageFilterUInt32_Size2 = ::AverageFilterUInt32_Size2;
using AverageFilterUInt32_Size3 = ::AverageFilterUInt32_Size3;
using AverageFilterUInt32_Size4 = ::AverageFilterUInt32_Size4;
using AverageFilterUInt32_Size5 = ::AverageFilterUInt32_Size5;
using AverageFilterFloat_Size5 = ::AverageFilterFloat_Size5;

// ============================================================================
// 低通滤波器
// ============================================================================

/// 数字低通滤波器基础实现
template <typename T>
using DigitalLPF = ::DigitalLPF<T>;

/// 低通滤波器（采样周期恒定）
/// 适用于采样周期恒定的场景，计算效率更高
template <typename T>
using LowPassFilterConstDt = ::LowPassFilterConstDt<T>;

/// 低通滤波器（采样周期可变）
/// 适用于采样周期不固定的场景
template <typename T>
using LowPassFilter = ::LowPassFilter<T>;

/// 数字双二阶滤波器
template <typename T>
using DigitalBiquadFilter = ::DigitalBiquadFilter<T>;

/// 二阶低通滤波器
template <typename T>
using LowPassFilter2p = ::LowPassFilter2p<T>;

// ============================================================================
// 陷波滤波器
// ============================================================================

/// 陷波滤波器
template <typename T>
using NotchFilter = ::NotchFilter<T>;

/// 谐波陷波滤波器
template <typename T>
using HarmonicNotchFilter = ::HarmonicNotchFilter<T>;

// 常用陷波滤波器类型别名
using NotchFilterFloat = ::NotchFilterFloat;
using NotchFilterVector2f = ::NotchFilterVector2f;
using NotchFilterVector3f = ::NotchFilterVector3f;

// ============================================================================
// 导数滤波器
// ============================================================================

/// 导数滤波器
template <typename T, uint8_t FILTER_SIZE>
using DerivativeFilter = ::DerivativeFilter<T, FILTER_SIZE>;

// 常用导数滤波器类型别名
using DerivativeFilterFloat_Size7 = ::DerivativeFilterFloat_Size7;

// ============================================================================
// 众数滤波器
// ============================================================================

/// 众数滤波器（Mode Filter）
template <typename T, uint8_t FILTER_SIZE>
using ModeFilter = ::ModeFilter<T, FILTER_SIZE>;

// 常用众数滤波器类型别名
using ModeFilterUInt16_Size3 = ::ModeFilterUInt16_Size3;
using ModeFilterUInt16_Size4 = ::ModeFilterUInt16_Size4;
using ModeFilterUInt16_Size5 = ::ModeFilterUInt16_Size5;
using ModeFilterInt16_Size3 = ::ModeFilterInt16_Size3;
using ModeFilterInt16_Size4 = ::ModeFilterInt16_Size4;
using ModeFilterInt16_Size5 = ::ModeFilterInt16_Size5;

// ============================================================================
// 转换率限制器
// ============================================================================

/// 转换率限制器（Slew Rate Limiter）
/// 用于限制信号变化速率
using SlewLimiter = ::SlewLimiter;

// ============================================================================
// Butterworth滤波器
// ============================================================================

/// 二阶Butterworth滤波器
template <typename T>
using Butter2 = ::Butter2<T>;

}  // namespace rm::modules

#endif  // LIBRM_MODULES_DSP_FILTERS_HPP