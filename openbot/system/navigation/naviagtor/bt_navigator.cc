/*
 * Copyright 2024 The OpenRobotic Beginner Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "openbot/system/navigation/naviagtor/bt_navigator.hpp"

namespace openbot {
namespace system { 
namespace navigation { 

BtNavigator::BtNavigator()
{
    const std::vector<std::string> plugin_libs = {
        "nav_compute_path_to_pose_action_bt_node",
        "nav_compute_path_through_poses_action_bt_node",
        "nav_smooth_path_action_bt_node",
        "nav_follow_path_action_bt_node",
        "nav_spin_action_bt_node",
        "nav_wait_action_bt_node",
        "nav_assisted_teleop_action_bt_node",
        "nav_back_up_action_bt_node",
        "nav_drive_on_heading_bt_node",
        "nav_clear_costmap_service_bt_node",
        "nav_is_stuck_condition_bt_node",
        "nav_goal_reached_condition_bt_node",
        "nav_initial_pose_received_condition_bt_node",
        "nav_goal_updated_condition_bt_node",
        "nav_globally_updated_goal_condition_bt_node",
        "nav_is_path_valid_condition_bt_node",
        "nav_reinitialize_global_localization_service_bt_node",
        "nav_rate_controller_bt_node",
        "nav_distance_controller_bt_node",
        "nav_speed_controller_bt_node",
        "nav_truncate_path_action_bt_node",
        "nav_truncate_path_local_action_bt_node",
        "nav_goal_updater_node_bt_node",
        "nav_recovery_node_bt_node",
        "nav_pipeline_sequence_bt_node",
        "nav_round_robin_node_bt_node",
        "nav_transform_available_condition_bt_node",
        "nav_time_expired_condition_bt_node",
        "nav_path_expiring_timer_condition",
        "nav_distance_traveled_condition_bt_node",
        "nav_single_trigger_bt_node",
        "nav_goal_updated_controller_bt_node",
        "nav_is_battery_low_condition_bt_node",
        "nav_navigate_through_poses_action_bt_node",
        "nav_navigate_to_pose_action_bt_node",
        "nav_remove_passed_goals_action_bt_node",
        "nav_planner_selector_bt_node",
        "nav_controller_selector_bt_node",
        "nav_goal_checker_selector_bt_node",
        "nav_controller_cancel_bt_node",
        "nav_path_longer_on_approach_bt_node",
        "nav_wait_cancel_bt_node",
        "nav_spin_cancel_bt_node",
        "nav_assisted_teleop_cancel_bt_node",
        "nav_back_up_cancel_bt_node",
        "nav_drive_on_heading_cancel_bt_node",
        "nav_is_battery_charging_condition_bt_node"
  };
}

}  // namespace navigation
}  // namespace system
}  // namespace openbot
