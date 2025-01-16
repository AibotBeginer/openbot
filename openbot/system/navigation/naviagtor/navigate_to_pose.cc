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

#include "openbot/system/navigation/naviagtor/navigate_to_pose.hpp"

#include "behaviortree_cpp/utils/shared_library.h"
#include "openbot/common/utils/logging.hpp"
#include "openbot/system/navigation/common/library_config.hpp"

namespace openbot {
namespace system { 
namespace navigation { 

NavigateToPoseNavigator::NavigateToPoseNavigator(
    const std::shared_ptr<apollo::cyber::Node>& node, 
    const openbot::navigation::NavigationConfig* config)
    : node_{node},
      config_{config}
{
    auto plugin_libs = GetDefaultPluginLibraries();
    bt_ = std::make_unique<behavior_tree::BehaviorTreeEngine>(plugin_libs);
     // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();
    auto bt_filename = config_->behavior_tree().default_xml_behavior_trees();
    tree_ = bt_->CreateTreeFromFile(bt_filename, blackboard_);

    AINFO << "Init navigate_to_pose navigator successed.";
}

void NavigateToPoseNavigator::SetTargetGoalPose(
    const std::shared_ptr<common::geometry_msgs::PoseStamped>& pose)
{
    
}

void NavigateToPoseNavigator::ExecuteCallback()
{


    // auto is_canceling = [&]() {
    //     // if (action_server_ == nullptr) {
    //     //     LOG(WARNING) << "Action server unavailable. Canceling.";
    //     //     return true;
    //     // }
    //     // if (!action_server_->IsServerActive()) {
    //     //     LOG(INFO) << "Action server is inactive. Canceling.";
    //     //     return true;
    //     // }
    //     // return action_server_->IsCancelRequested();

    //     return false;
    // };

    // auto on_loop = [&]() {
    //     // if (action_server_->IsPreemptRequested() && on_preempt_callback_) {
    //     //     on_preempt_callback_(action_server_->GetPendingRequest());
    //     // }
    //     // on_loop_callback_();
    // };

    // // Execute the BT that was previously created in the configure step
    // behavior_tree::BtStatus rc = bt_->Run(&tree_, on_loop, is_canceling, bt_loop_duration_);

    // // Make sure that the Bt is not in a running state from a previous execution
    // // note: if all the ControlNodes are implemented correctly, this is not needed.
    // bt_->HaltAllActions(tree_);

    // // Give server an opportunity to populate the result message or simple give
    // // an indication that the action is complete.
    // auto result = std::make_shared<typename ActionT::Response>();
    // on_completion_callback_(result, rc);

    // switch (rc) 
    // {
    //     case behavior_tree::BtStatus::SUCCEEDED:
    //         LOG(INFO) << "Goal succeeded";
    //         // action_server_->succeeded_current(result);
    //         break;

    //     case behavior_tree::BtStatus::FAILED:
    //         LOG(INFO) << "Goal failed";
    //         // action_server_->terminate_current(result);
    //         break;

    //     case behavior_tree::BtStatus::CANCELED:
    //         LOG(INFO) << "Goal canceled";
    //         // action_server_->terminate_all(result);
    //         break;
    // }
}

bool NavigateToPoseNavigator::InitConfig(const std::vector<std::string>& plugin_libraries)
{
    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard_->set<std::shared_ptr<::apollo::cyber::Node>>("node", node_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);

    // Create the class that registers our custom nodes and executes the BT
    // bt_ = std::make_unique<behavior_tree::BehaviorTreeEngine>(plugin_libraries);
    plugin_lib_names_ = plugin_libraries;
    return true;
}

std::string NavigateToPoseNavigator::GetDefaultBTFilepath()
{
    return config_->behavior_tree().default_xml_behavior_trees();
}

bool NavigateToPoseNavigator::LoadBehaviorTree(const std::string& bt_xml_filename)
{
    // // Empty filename is default for backward compatibility
    // auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    // // Use previous BT if it is the existing one
    // if (current_bt_xml_filename_ == filename) {
    //     LOG(INFO) << "BT will not be reloaded as the given xml is already loaded";
    //     return true;
    // }
    
    // // Read the input BT XML from the specified file into a string
    // std::ifstream xml_file(filename);

    // if (!xml_file.good()) {
    //     LOG(ERROR) << "Couldn't open input XML file: " << filename;
    //     return false;
    // }


    // auto xml_string = std::string(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());


    // AINFO << "xml_string:" << xml_string;

    // Create the Behavior Tree from the XML input
    try {

        AINFO << "-------------------------------------1";
        // tree_ = bt_->CreateTreeFromText(xml_string, blackboard_);
        AINFO << "-------------------------------------2";
        // for (auto& subtree : tree_.subtrees) {
        //     auto& blackboard = subtree->blackboard;
        //     blackboard->set("node", node_);
        //     blackboard->set<std::shared_ptr<::apollo::cyber::Node>>("node", node_);
        //     blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
        //     blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        // }
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception when loading BT: " << e.what();
        return false;
    }

    // current_bt_xml_filename_ = filename;
    return true;
}

std::vector<std::string> NavigateToPoseNavigator::GetDefaultPluginLibraries()
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
        "nav_path_expiring_timer_condition_bt_node",
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
        "nav_is_battery_charging_condition_bt_node",
        "nav_dummy_nodes_bt_node"
    };

    return plugin_libs;
}

}  // namespace navigation
}  // namespace system
}  // namespace openbot
