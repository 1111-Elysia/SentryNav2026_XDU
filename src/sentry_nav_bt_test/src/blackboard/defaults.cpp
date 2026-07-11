#include "sentry_nav_bt_test/blackboard/defaults.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace sentry_nav_bt_test::blackboard
{

void initializeDefaults(const BT::Blackboard::Ptr &bb)
{
    bb->set("bt_loop_duration", std::chrono::milliseconds(10));
    bb->set("wait_for_service_timeout", std::chrono::milliseconds(1000));
    bb->set("server_timeout", std::chrono::milliseconds(1000));
    bb->set("initial_pose_received", false);
    bb->set("game_status_received", false);
    bb->set("game_status_connected_logged", 0);
    bb->set("last_referee_tx_ok", false);
    bb->set("last_posture_request_target", -1);
    bb->set("last_posture_request_sent", false);
    bb->set("last_posture_request_tx_ok", false);
    bb->set("last_posture_request_confirmed", false);
    bb->set("last_posture_request_pending", false);
    bb->set("last_posture_request_result", std::string("idle"));
    bb->set("last_posture_request_time_s", -1.0);
    bb->set("posture_switch_cooldown_ms", 5000);
    bb->set("ul_initialized", 0);
    bb->set("uc_initialized", 0);
    bb->set("ul_retreat_active", 0);
    bb->set("ul_center_ready", 0);
    bb->set("uc_supply_active", 0);
    bb->set("uc_outpost_active", 0);
    bb->set("uc_chase_active", 0);
    bb->set("uc_normal_posture", 3);
    bb->set("power_management_shooter_output", true);
    bb->set("power_heat_data_received", false);
    bb->set("shooter_17mm_barrel_heat", 0);
    bb->set("shooter_17mm_barrel_heat_prev", 0);
    bb->set("shooter_17mm_barrel_heat_delta", 0);
    bb->set("shooter_17mm_heat_firing", 0);
    bb->set("center_gain_point_occupancy_status", 0);
    bb->set("ul_center_goal_name", std::string("center_point"));
    bb->set("ul_center_arrive_distance_threshold", 0.10);
    bb->set("ul_center_hold_distance_threshold", 0.50);
    bb->set("ul_center_hold_exit_distance_threshold", 0.55);
    bb->set("uc_fortress_hold_active", 0);
    bb->set("uc_fortress_goal_name", std::string("fortress"));
    bb->set("uc_fortress_hold_distance_threshold", 0.25);
    bb->set("uc_fortress_hold_exit_distance_threshold", 0.30);
    bb->set("fortress_gain_point_occupancy_status", 0);
    bb->set("outpost_gain_point_occupancy_status", 0);
    bb->set("base_gain_point_occupied", 0);
    bb->set("map_command_received", false);
    bb->set("sentry_info_received", false);
    bb->set("ul_pose_stale_timeout_s", 0.50);
    bb->set("waypoint_now_valid", false);
    bb->set("rfid_status", uint32_t{0});
    bb->set("rfid_status_2", uint8_t{0});
    bb->set("rfid_ally_fortress_detected", 0);
    bb->set("rfid_supply_zone_bit19_detected", 0);
    bb->set("rfid_supply_zone_bit20_detected", 0);
    bb->set("rfid_supply_zone_detected", 0);
    bb->set("hurt_armor_id", -1);
    bb->set("hurt_armor", 0);
    bb->set("is_under_attack", 0);
    bb->set("in_rune_phase", 0);
    bb->set("simple_nav_initialized", 0);
    bb->set("simple_nav_completed", 0);
    bb->set("runtime_effective_goal_name", std::string("init"));
    bb->set("runtime_move_posture", 3);
    bb->set("runtime_wait_posture", 1);
    bb->set("runtime_controller", std::string("FollowPath"));
    bb->set("runtime_reach_threshold", 0.25);
    bb->set("runtime_wait_time_threshold", 5.0);
    bb->set("runtime_use_custom_pose", false);
}

}  // namespace sentry_nav_bt_test::blackboard
