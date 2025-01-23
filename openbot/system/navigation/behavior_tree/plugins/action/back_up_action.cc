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

#include "openbot/system/navigation/behavior_tree/plugins/action/back_up_action.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

BackUpAction::BackUpAction(
  const std::string& xml_tag_name,
  const std::string& action_name,
  const BT::NodeConfiguration& conf)
: BtActionNode<openbot::navigation::BackUp>(xml_tag_name, action_name, conf),
  initialized_(false)
{
}

void BackUpAction::Initialize()
{
    double dist;
    getInput("backup_dist", dist);
    double speed;
    getInput("backup_speed", speed);
    double time_allowance;
    getInput("time_allowance", time_allowance);

    // // Populate the input message
    // goal_.target.x = dist;
    // goal_.target.y = 0.0;
    // goal_.target.z = 0.0;
    // goal_.speed = speed;
    // goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
    initialized_ = true;
}

void BackUpAction::OnTick()
{
    if (!initialized_) {
        Initialize();
    }

    IncrementRecoveryCount();
}

BT::NodeStatus BackUpAction::OnSuccess()
{
    // setOutput("error_code_id", ActionResult::NONE);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BackUpAction::OnAborted()
{
    // setOutput("error_code_id", result_.result->error_code);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus BackUpAction::OnCancelled()
{
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
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<openbot::system::navigation::behavior_tree::BackUpAction>(
            name, "backup", config);
    };

    factory.registerBuilder<openbot::system::navigation::behavior_tree::BackUpAction>("BackUp", builder);
}