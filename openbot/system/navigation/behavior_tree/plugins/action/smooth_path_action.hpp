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

#include "openbot/common/io/msgs.hpp"
#include "openbot/system/navigation/behavior_tree/bt_action_node.hpp"
#include "openbot/system/navigation/proto/smooth_path.pb.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BtActionNode class that wraps openbot::navigation::SmoothPath
 */
class SmoothPathAction : public BtActionNode<openbot::navigation::SmoothPath>
{
  using Action = openbot::navigation::SmoothPath;
  using ActionResult = Action::Response;

public:
    /**
     * @brief A constructor for nav2_behavior_tree::SmoothPathAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    SmoothPathAction(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief Function to perform some user-defined operation on tick
     */
    void OnTick() override;

    /**
     * @brief Function to perform some user-defined operation upon successful completion of the action
     */
    BT::NodeStatus OnSuccess() override;

    /**
     * @brief Function to perform some user-defined operation upon abortion of the action
     */
    BT::NodeStatus OnAborted() override;

    /**
     * @brief Function to perform some user-defined operation upon cancellation of the action
     */
    BT::NodeStatus OnCancelled() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<common::nav_msgs::Path>("unsmoothed_path", "Path to be smoothed"),
            BT::InputPort<double>("max_smoothing_duration", 3.0, "Maximum smoothing duration"),
            BT::InputPort<bool>(
                "check_for_collisions", false,
                "If true collision check will be performed after smoothing"),
            BT::InputPort<std::string>("smoother_id", ""),
            BT::OutputPort<common::nav_msgs::Path>("smoothed_path", "Path smoothed by SmootherServer node"),
            BT::OutputPort<double>("smoothing_duration", "Time taken to smooth path"),
            BT::OutputPort<bool>("was_completed", "True if smoothing was not interrupted by time limit"),
            BT::OutputPort<Action::ErrorCode>("error_code_id", "The smooth path error code"),
        });
    }
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot