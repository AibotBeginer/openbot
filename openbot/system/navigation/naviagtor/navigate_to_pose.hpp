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

#pragma once

#include "cyber/cyber.h"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "openbot/common/io/msgs.hpp"
#include "openbot/common/macros.hpp"
#include "openbot/system/navigation/behavior_tree/behavior_tree_engine.hpp"
#include "openbot/system/navigation/proto/navigate_to_pose.pb.h"
#include "openbot/system/navigation/proto/bt_navigator.pb.h"

namespace openbot {
namespace system { 
namespace navigation { 

/**
 * @class NavigateToPoseNavigator
 * @brief A navigator for navigating to a specified pose
 */
class NavigateToPoseNavigator
{
public:
    using ActionT = openbot::navigation::NavigateToPose;

    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(NavigateToPoseNavigator);

    /**
     * @brief A constructor for NavigateToPoseNavigator
     */
    NavigateToPoseNavigator(
        const std::shared_ptr<apollo::cyber::Node>& node, 
        const openbot::navigation::NavigationConfig* config);

    /**
     * @brief A subscription and callback to handle the topic-based goal published
     * from rviz
     * @param pose Pose received via atopic
     */
    void SetTargetGoalPose(const std::shared_ptr<common::geometry_msgs::PoseStamped>& pose);

    /**
     * @brief Action server callback
     */
    void ExecuteCallback();

private:

    /**
     * @brief Init config plugins
     * 
     * @param plugin_libraries 
     * @return true 
     * @return false 
     */
    bool InitConfig(const std::vector<std::string>& plugin_libraries);
    
        /**
     * @brief Get navigator's default BT
     * @param node WeakPtr to the lifecycle node
     * @return string Filepath to default XML
     */
    std::string GetDefaultBTFilepath();

    /**
     * @brief Get action name for this navigator
     * @return string Name of action server
     */
    std::string GetName() { return std::string("navigate_to_pose"); }

    /**
     * @brief Load behavior_tree config file
     * 
     * @param filename 
     * @return true 
     * @return false 
     */
    bool LoadBehaviorTree(const std::string& bt_xml_filename);

     /**
     * @brief Getter function for BT Blackboard
     * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
     */
    BT::Blackboard::Ptr GetBlackboard() const
    {
        return blackboard_;
    }

    /**
     * @brief Getter function for current BT XML filename
     * @return string Containing current BT XML filename
     */
    std::string GetCurrentBTFilename() const
    {
        return current_bt_xml_filename_;
    }

    /**
     * @brief Getter function for default BT XML filename
     * @return string Containing default BT XML filename
     */
    std::string GetDefaultBTFilename() const
    {
        return default_bt_xml_filename_;
    }

    /**
     * @brief Getter function for the current BT tree
     * @return BT::Tree Current behavior tree
     */
    const BT::Tree& GetTree() const
    {
        return tree_;
    }

    /**
     * @brief Get the Default Plugin Libraries object
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> GetDefaultPluginLibraries();

    // blackboard_id
    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;

    // config
    const openbot::navigation::NavigationConfig* config_;

    // goal_pose
    std::shared_ptr<common::geometry_msgs::PoseStamped> goal_pose_;

    // The wrapper class for the BT functionality
    std::unique_ptr<behavior_tree::BehaviorTreeEngine> bt_;
    // BT::BehaviorTreeFactory factory_;

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;

    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;

    // The XML file that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    // Libraries to pull plugins (BT Nodes) from
    std::vector<std::string> plugin_lib_names_;

    // A regular, non-spinning ROS node that we can use for calls to the action client
    std::shared_ptr<apollo::cyber::Node> node_;

    // Duration for each iteration of BT execution
    std::chrono::milliseconds bt_loop_duration_;

    // Default timeout value while waiting for response from a server
    std::chrono::milliseconds default_server_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;
};

}  // namespace navigation
}  // namespace system
}  // namespace openbot
