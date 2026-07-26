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
 * @file  librm/modules/keyboard_handler.hpp
 * @brief 键盘输入处理器
 */

#ifndef LIBRM_MODULES_KEYBOARD_HANDLER_HPP
#define LIBRM_MODULES_KEYBOARD_HANDLER_HPP

#include <etl/algorithm.h>
#include <etl/delegate.h>
#include <etl/vector.h>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief 键盘按键事件
 */
struct KeyEvent {
  /**
   * @brief 键盘按键事件类型
   */
  enum Type {
    kKeyDown,  ///< 按键按下
    kKeyUp     ///< 按键释放
  };
  u32 key;        ///< 触发的按键（单个键的掩码）
  u32 modifiers;  ///< 当前按下的modifier键组合
  Type type;      ///< 事件类型
};

/**
 * @brief 键盘输入处理器
 *
 * @tparam MaxBindings       最大绑定数量（KeyDown和KeyUp各自的上限）
 * @tparam MaxConcurrentKeys 最大同时按下的普通键数量
 * @tparam UseStdFunction    是否使用std::function作为回调类型（否则使用etl::delegate）
 */
template <usize MaxBindings = 32, usize MaxConcurrentKeys = 8, bool UseStdFunction = true>
class KeyboardHandler {
 public:
  using KeyCallback = std::conditional_t<UseStdFunction,                        //
                                         std::function<void(const KeyEvent&)>,  //
                                         etl::delegate<void(const KeyEvent&)>>;

  /**
   * @brief 构造函数
   * @param modifier_mask 所有modifier键的组合掩码，用于区分modifier键和普通键
   *
   * @example
   * ```cpp
   * // 假设Ctrl在bit16，Shift在bit17，Alt在bit18
   * constexpr u32 kAllModifiers = (1U << 16) | (1U << 17) | (1U << 18);
   * KeyboardHandler<32, 8> handler(kAllModifiers);
   * ```
   */
  explicit KeyboardHandler(u32 modifier_mask) : modifier_mask_(modifier_mask) {}

  ~KeyboardHandler() = default;

  // 允许拷贝
  KeyboardHandler(const KeyboardHandler&) = default;
  KeyboardHandler& operator=(const KeyboardHandler&) = default;

  // 禁止移动（etl容器不支持移动语义）
  KeyboardHandler(KeyboardHandler&&) = delete;
  KeyboardHandler& operator=(KeyboardHandler&&) = delete;

  /**
   * @brief 获取当前设置的modifier掩码
   * @return modifier掩码
   */
  [[nodiscard]] u32 modifier_mask() const { return modifier_mask_; }

  /**
   * @brief 设置modifier掩码
   * @param mask 新的modifier掩码
   * @return 返回自身引用以支持链式调用
   */
  KeyboardHandler& SetModifierMask(u32 mask) {
    modifier_mask_ = mask;
    return *this;
  }

  /**
   * @brief 注册按键按下回调（无modifier）
   * @param key 要监听的按键掩码（单个键）
   * @param callback 回调函数
   * @return 返回自身引用以支持链式调用
   */
  KeyboardHandler& OnKeyDown(u32 key, KeyCallback callback) { return OnKeyDown(key, 0, callback); }

  /**
   * @brief 注册按键按下回调（带modifier）
   * @param key 要监听的按键掩码（单个键）
   * @param required_modifiers 要求的modifier键组合
   * @param callback 回调函数
   * @return 返回自身引用以支持链式调用
   */
  KeyboardHandler& OnKeyDown(u32 key, u32 required_modifiers, KeyCallback callback) {
    if (!key_down_bindings_.full()) {
      key_down_bindings_.push_back({key, required_modifiers, callback});
      // 按modifier数量降序排序，确保组合键优先匹配
      SortBindings(key_down_bindings_);
    }
    return *this;
  }

  /**
   * @brief 注册按键释放回调（无modifier）
   * @param key 要监听的按键掩码（单个键）
   * @param callback 回调函数
   * @return 返回自身引用以支持链式调用
   */
  KeyboardHandler& OnKeyUp(u32 key, KeyCallback callback) { return OnKeyUp(key, 0, callback); }

  /**
   * @brief 注册按键释放回调（带modifier）
   * @param key 要监听的按键掩码（单个键）
   * @param required_modifiers 要求的modifier键组合（按下该键时的modifier状态）
   * @param callback 回调函数
   * @return 返回自身引用以支持链式调用
   */
  KeyboardHandler& OnKeyUp(u32 key, u32 required_modifiers, KeyCallback callback) {
    if (!key_up_bindings_.full()) {
      key_up_bindings_.push_back({key, required_modifiers, callback});
      SortBindings(key_up_bindings_);
    }
    return *this;
  }

  /**
   * @brief 更新键盘状态并触发相应的回调
   * @param keycode 当前的keycode，每一位对应一个键的按下状态
   *
   * @details 处理逻辑：
   *          1. 计算新按下的键和新释放的键
   *          2. 处理modifier键状态更新
   *          3. 对于新按下的普通键，检查是否满足组合键条件
   *          4. 触发相应的KeyDown/KeyUp回调
   */
  void Update(u32 keycode) {
    u32 regular_mask = ~modifier_mask_;

    u32 current_modifiers = keycode & modifier_mask_;
    u32 current_regular_keys = keycode & regular_mask;

    u32 prev_regular_keys = prev_keycode_ & regular_mask;

    // 检测新按下的普通键
    u32 newly_pressed_regular = current_regular_keys & ~prev_regular_keys;
    // 检测新释放的普通键
    u32 newly_released_regular = prev_regular_keys & ~current_regular_keys;

    // 处理新按下的普通键
    for (u32 bit = 0; bit < 32; ++bit) {
      u32 key_mask = 1U << bit;

      // 跳过modifier键
      if (key_mask & modifier_mask_) {
        continue;
      }

      if (newly_pressed_regular & key_mask) {
        // 普通键是新按下的，使用当前modifier状态来判断组合键
        HandleKeyDown(key_mask, current_modifiers);
      }
    }

    // 处理新释放的普通键
    for (u32 bit = 0; bit < 32; ++bit) {
      u32 key_mask = 1U << bit;

      if (key_mask & modifier_mask_) {
        continue;
      }

      if (newly_released_regular & key_mask) {
        // 释放键时，使用该键按下时记录的modifier状态
        HandleKeyUp(key_mask);
      }
    }

    // 更新按键按下时的modifier状态记录
    UpdatePressedKeyModifiers(newly_pressed_regular, current_modifiers);
    ClearReleasedKeyModifiers(newly_released_regular);

    prev_keycode_ = keycode;
  }

  /**
   * @brief 重置键盘处理器状态
   */
  void Reset() {
    prev_keycode_ = 0;
    pressed_key_modifiers_.clear();
  }

  /**
   * @brief 清除所有已注册的回调
   */
  void ClearBindings() {
    key_down_bindings_.clear();
    key_up_bindings_.clear();
  }

 private:
  /**
   * @brief 按键绑定结构体
   */
  struct KeyBinding {
    u32 key;                 ///< 目标按键掩码
    u32 required_modifiers;  ///< 要求的modifier键组合（0表示无modifier要求）
    KeyCallback callback;    ///< 回调函数
  };

  /**
   * @brief 按下键时的modifier状态记录
   */
  struct PressedKeyState {
    u32 key;        ///< 按键掩码
    u32 modifiers;  ///< 按下时的modifier状态
  };

  using BindingVector = etl::vector<KeyBinding, MaxBindings>;
  using PressedKeyVector = etl::vector<PressedKeyState, MaxConcurrentKeys>;

  /**
   * @brief 按modifier数量降序排序绑定列表
   * @param bindings 绑定列表
   *
   * @details 这确保了更具体的组合键（更多modifier）优先被匹配
   */
  static void SortBindings(BindingVector& bindings) {
    etl::sort(bindings.begin(), bindings.end(), [](const KeyBinding& a, const KeyBinding& b) {
      return CountBits(a.required_modifiers) > CountBits(b.required_modifiers);
    });
  }

  /**
   * @brief 计算一个数中1的个数
   */
  static u32 CountBits(u32 n) {
    u32 count = 0;
    while (n) {
      count += n & 1;
      n >>= 1;
    }
    return count;
  }

  /**
   * @brief 处理按键按下事件
   * @param key 按下的键
   * @param current_modifiers 当前的modifier状态
   */
  void HandleKeyDown(u32 key, u32 current_modifiers) {
    bool matched = false;

    // 查找匹配的绑定（已按modifier数量降序排序）
    for (auto& binding : key_down_bindings_) {
      if (binding.key != key) {
        continue;
      }

      // 检查modifier是否精确匹配
      // 要求的modifier必须全部按下，且不能有额外的modifier
      if (binding.required_modifiers == current_modifiers) {
        // 精确匹配
        KeyEvent event{key, current_modifiers, KeyEvent::Type::kKeyDown};
        binding.callback(event);
        matched = true;
        break;  // 只触发第一个匹配的绑定（最具体的）
      }
    }

    // 如果没有精确匹配，并且当前有modifier键按下，
    // 则不触发无modifier的单键回调（模仿桌面OS行为）
    // 如果没有modifier键按下，且有无modifier的绑定，则触发它
    if (!matched && current_modifiers == 0) {
      for (auto& binding : key_down_bindings_) {
        if (binding.key == key && binding.required_modifiers == 0) {
          KeyEvent event{key, 0, KeyEvent::Type::kKeyDown};
          binding.callback(event);
          break;
        }
      }
    }
  }

  /**
   * @brief 处理按键释放事件
   * @param key 释放的键
   */
  void HandleKeyUp(u32 key) {
    // 查找该键按下时的modifier状态
    u32 modifiers_when_pressed = 0;
    for (const auto& state : pressed_key_modifiers_) {
      if (state.key == key) {
        modifiers_when_pressed = state.modifiers;
        break;
      }
    }

    bool matched = false;

    // 查找匹配的绑定
    for (auto& binding : key_up_bindings_) {
      if (binding.key != key) {
        continue;
      }

      if (binding.required_modifiers == modifiers_when_pressed) {
        KeyEvent event{key, modifiers_when_pressed, KeyEvent::Type::kKeyUp};
        binding.callback(event);
        matched = true;
        break;
      }
    }

    if (!matched && modifiers_when_pressed == 0) {
      for (auto& binding : key_up_bindings_) {
        if (binding.key == key && binding.required_modifiers == 0) {
          KeyEvent event{key, 0, KeyEvent::Type::kKeyUp};
          binding.callback(event);
          break;
        }
      }
    }
  }

  /**
   * @brief 记录新按下键的modifier状态
   */
  void UpdatePressedKeyModifiers(u32 newly_pressed, u32 modifiers) {
    for (u32 bit = 0; bit < 32; ++bit) {
      u32 key_mask = 1U << bit;
      if ((newly_pressed & key_mask) && !pressed_key_modifiers_.full()) {
        pressed_key_modifiers_.push_back({key_mask, modifiers});
      }
    }
  }

  /**
   * @brief 清除已释放键的modifier状态记录
   */
  void ClearReleasedKeyModifiers(u32 released) {
    auto it = pressed_key_modifiers_.begin();
    while (it != pressed_key_modifiers_.end()) {
      if (it->key & released) {
        it = pressed_key_modifiers_.erase(it);
      } else {
        ++it;
      }
    }
  }

  u32 modifier_mask_;                       ///< 所有modifier键的组合掩码
  u32 prev_keycode_{0};                     ///< 上一次的keycode
  BindingVector key_down_bindings_;         ///< 按键按下回调绑定
  BindingVector key_up_bindings_;           ///< 按键释放回调绑定
  PressedKeyVector pressed_key_modifiers_;  ///< 记录每个按下键的modifier状态
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_KEYBOARD_HANDLER_HPP
