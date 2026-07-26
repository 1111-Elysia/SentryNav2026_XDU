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
 * @file  librm/device/referee/protocol_new_v200.hpp
 * @brief 裁判系统通信协议V2.0.0（2026-06-26）
 */

#ifndef LIBRM_DEVICE_REFEREE_PROTOCOL_NEW_V200_HPP
#define LIBRM_DEVICE_REFEREE_PROTOCOL_NEW_V200_HPP

#include "protocol.hpp"

#include <cstddef>

#include <mapbox/eternal.hpp>

namespace rm::device {

template <>
struct RefereeCmdId<RefereeRevision::kNewV200> {
  constexpr static u16 kGameStatus = 0x1;
  constexpr static u16 kGameResult = 0x2;
  constexpr static u16 kGameRobotHp = 0x3;
  constexpr static u16 kEventData = 0x101;
  constexpr static u16 kRefereeWarning = 0x104;
  constexpr static u16 kDartInformation = 0x105;
  constexpr static u16 kRobotStatus = 0x201;
  constexpr static u16 kPowerHeatData = 0x202;
  constexpr static u16 kRobotPos = 0x203;
  constexpr static u16 kBuff = 0x204;
  constexpr static u16 kHurtData = 0x206;
  constexpr static u16 kShootData = 0x207;
  constexpr static u16 kProjectileAllowance = 0x208;
  constexpr static u16 kRfidStatus = 0x209;
  constexpr static u16 kDartClientCmd = 0x20a;
  constexpr static u16 kGroundRobotPosition = 0x20b;
  constexpr static u16 kRadarMarkData = 0x20c;
  constexpr static u16 kSentryInfo = 0x20d;
  constexpr static u16 kRadarInfo = 0x20e;
  constexpr static u16 kCustomRobotData = 0x302;   // 图传链路
  constexpr static u16 kMapCommand = 0x303;
  constexpr static u16 kRadarInfoToClient = 0x305;
  constexpr static u16 kCustomClientData = 0x306;  // 图传链路
  constexpr static u16 kMapData = 0x307;
  constexpr static u16 kCustomInfo = 0x308;
  constexpr static u16 kRobotCustomData = 0x309;   // 图传链路
  constexpr static u16 kRobotCustomData2 = 0x310;  // 图传链路
  constexpr static u16 kCustomControl = 0x311;     // 图传链路
  constexpr static u16 kSetVtChannel = 0x0f01;
  constexpr static u16 kQueryVtChannel = 0x0f02;
};

#pragma pack(push, 1)

template <>
struct RefereeProtocol<RefereeRevision::kNewV200> {
  struct {
    u8 game_type : 4;
    u8 game_progress : 4;
    u16 stage_remain_time;
    u64 SyncTimeStamp;
  } game_status;
  struct {
    u8 winner;
  } game_result;
  struct {
    u16 ally_1_robot_HP;
    u16 ally_2_robot_HP;
    u16 ally_3_robot_HP;
    u16 ally_4_robot_HP;
    i16 damage_difference;
    u16 ally_7_robot_HP;
    u16 ally_outpost_HP;
    u16 ally_base_HP;
    u16 enemy_outpost_HP;
    u16 enemy_base_HP;
  } game_robot_HP;
  struct {
    // 0x0101 的位定义在 V1.3.0 中有修订，布局仍保持 uint32_t。
    u32 event_data;
  } event_data;
  struct {
    u8 level;
    u8 offending_robot_id;
    u8 count;
  } referee_warning;
  struct {
    u8 dart_remaining_time;
    u16 dart_info;
  } dart_info;
  struct {
    u8 robot_id;
    u8 robot_level;
    u16 current_HP;
    u16 maximum_HP;
    u16 shooter_barrel_cooling_value;
    u16 shooter_barrel_heat_limit;
    u16 chassis_power_limit;
    f32 bullet_speed_limit;
    u8 power_management_gimbal_output : 1;
    u8 power_management_chassis_output : 1;
    u8 power_management_shooter_output : 1;
  } robot_status;
  struct {
    u16 reserved;
    u16 reserved_2;
    f32 reserved_3;
    u16 buffer_energy;
    u16 shooter_17mm_1_barrel_heat;
    u16 shooter_42mm_barrel_heat;
  } power_heat_data;
  struct {
    f32 x;
    f32 y;
    f32 angle;
  } robot_pos;
  struct {
    u8 recovery_buff;
    u16 cooling_buff;
    u8 defence_buff;
    u8 vulnerability_buff;
    u16 attack_buff;
    u8 remaining_energy;
  } buff;
  struct {
    u8 armor_id : 4;
    u8 HP_deduction_reason : 4;
  } hurt_data;
  struct {
    u8 bullet_type;
    u8 shooter_number;
    u8 launching_frequency;
    f32 initial_speed;
  } shoot_data;
  struct {
    u16 projectile_allowance_17mm;
    u16 projectile_allowance_42mm;
    u16 remaining_gold_coin;
    u16 projectile_allowance_fortress;
  } projectile_allowance;
  struct {
    u32 rfid_status;
    u8 rfid_status_2;
  } rfid_status;
  struct {
    u8 dart_launch_opening_status;
    u8 reserved;
    u16 target_change_time;
    u16 latest_launch_cmd_time;
  } dart_client_cmd;
  struct {
    f32 hero_x;
    f32 hero_y;
    f32 engineer_x;
    f32 engineer_y;
    f32 standard_3_x;
    f32 standard_3_y;
    f32 standard_4_x;
    f32 standard_4_y;
    f32 reserved;
    f32 reserved_2;
  } ground_robot_position;
  struct {
    u16 mark_progress;
  } radar_mark_data;
  struct {
    u32 sentry_info;
    u16 sentry_info_2;
    u64 sentry_info_3;
  } sentry_info;
  struct {
    u8 radar_info;
  } radar_info;
  struct {
    u8 data[30];
  } custom_robot_data;
  struct {
    u8 data[30];
  } robot_custom_data;
  struct {
    u8 data[300];
  } robot_custom_data_2;
  struct {
    u8 data[30];
  } custom_control;
  struct {
    f32 target_position_x;
    f32 target_position_y;
    u8 cmd_keyboard;
    u8 target_robot_id;
    u16 cmd_source;
  } map_command;
  struct {
    u16 opponent_hero_position_x;
    u16 opponent_hero_position_y;
    u16 opponent_engineer_position_x;
    u16 opponent_engineer_position_y;
    u16 opponent_infantry_3_position_x;
    u16 opponent_infantry_3_position_y;
    u16 opponent_infantry_4_position_x;
    u16 opponent_infantry_4_position_y;
    u16 opponent_aerial_position_x;
    u16 opponent_aerial_position_y;
    u16 opponent_sentry_position_x;
    u16 opponent_sentry_position_y;
    u16 ally_hero_position_x;
    u16 ally_hero_position_y;
    u16 ally_engineer_position_x;
    u16 ally_engineer_position_y;
    u16 ally_infantry_3_position_x;
    u16 ally_infantry_3_position_y;
    u16 ally_infantry_4_position_x;
    u16 ally_infantry_4_position_y;
    u16 ally_aerial_position_x;
    u16 ally_aerial_position_y;
    u16 ally_sentry_position_x;
    u16 ally_sentry_position_y;
  } radar_info_to_client;
  struct {
    u8 intention;
    u16 start_position_x;
    u16 start_position_y;
    i8 delta_x[49];
    i8 delta_y[49];
    u16 sender_id;
  } map_data;
  struct {
    u16 sender_id;
    u16 receiver_id;
    u8 user_data[30];
  } custom_info;
  struct {
    u8 channel;
  } set_vt_channel;
  struct {
    u8 channel;
  } query_vt_channel;
  struct {
    u16 key_value;
    u16 x_position : 12;
    u16 mouse_left : 4;
    u16 y_position : 12;
    u16 mouse_right : 4;
    u16 reserved;
  } custom_client_data;
};

#pragma pack(pop)

static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kNewV200>::game_robot_HP)) == 20);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kNewV200>::robot_status)) == 17);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kNewV200>::radar_mark_data)) == 2);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kNewV200>::sentry_info)) == 14);
static_assert(sizeof(decltype(RefereeProtocol<RefereeRevision::kNewV200>::map_command)) == 12);

template <>
struct RefereeProtocolMemoryMap<RefereeRevision::kNewV200> {
  // clang-format off
  static MAPBOX_ETERNAL_CONSTEXPR const auto map = mapbox::eternal::map<u16, usize>({
  {RefereeCmdId<RefereeRevision::kNewV200>::kGameStatus, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, game_status)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kGameResult, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, game_result)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kGameRobotHp, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, game_robot_HP)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kEventData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, event_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRefereeWarning, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, referee_warning)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kDartInformation, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, dart_info)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRobotStatus, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, robot_status)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kPowerHeatData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, power_heat_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRobotPos, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, robot_pos)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kBuff, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, buff)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kHurtData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, hurt_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kShootData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, shoot_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kProjectileAllowance, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, projectile_allowance)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRfidStatus, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, rfid_status)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kDartClientCmd, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, dart_client_cmd)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kGroundRobotPosition, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, ground_robot_position)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRadarMarkData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, radar_mark_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kSentryInfo, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, sentry_info)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRadarInfo, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, radar_info)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kCustomRobotData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, custom_robot_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kMapCommand, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, map_command)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRadarInfoToClient, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, radar_info_to_client)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kCustomClientData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, custom_client_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kMapData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, map_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kCustomInfo, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, custom_info)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRobotCustomData, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, robot_custom_data)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kRobotCustomData2, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, robot_custom_data_2)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kCustomControl, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, custom_control)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kSetVtChannel, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, set_vt_channel)},
      {RefereeCmdId<RefereeRevision::kNewV200>::kQueryVtChannel, offsetof(RefereeProtocol<RefereeRevision::kNewV200>, query_vt_channel)}
  });
  // clang-format on
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_REFEREE_PROTOCOL_NEW_V200_HPP
