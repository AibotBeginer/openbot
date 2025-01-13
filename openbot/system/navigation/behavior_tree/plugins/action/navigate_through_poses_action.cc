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

#include "openbot/system/navigation/behavior_tree/plugins/action/navigate_through_poses_action.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string& xml_tag_name,
  const std::string& action_name,
  const BT::NodeConfiguration& conf)
: BtActionNode<openbot::navigation::NavigateThroughPoses>(xml_tag_name, action_name, conf)
{
}

void NavigateThroughPosesAction::OnTick()
{
    // if (!getInput("goals", goal_.poses)) {
    //         RCLCPP_ERROR(
    //         node_->get_logger(),
    //         "NavigateThroughPosesAction: goal not provided");
    //     return;
    // }
    // getInput("behavior_tree", goal_.behavior_tree);
}

BT::NodeStatus NavigateThroughPosesAction::OnSuccess()
{
    // setOutput("error_code_id", ActionResult::NONE);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus NavigateThroughPosesAction::OnAborted()
{
    // setOutput("error_code_id", result_.result->error_code);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateThroughPosesAction::OnCancelled()
{
    // Set empty error code, action was cancelled
    // setOutput("error_code_id", ActionResult::NONE);
    return BT::NodeStatus::SUCCESS;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
    {
        return std::make_unique<openbot::system::navigation::behavior_tree::NavigateThroughPosesAction>(
            name, "navigate_through_poses", config);
    };

    factory.registerBuilder<openbot::system::navigation::behavior_tree::NavigateThroughPosesAction>(
        "NavigateThroughPoses", builder);
}