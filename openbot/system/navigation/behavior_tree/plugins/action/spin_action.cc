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

#include "openbot/system/navigation/behavior_tree/plugins/action/spin_action.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

SpinAction::SpinAction(
  const std::string& xml_tag_name,
  const std::string& action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<openbot::navigation::Spin>(xml_tag_name, action_name, conf),
  initialized_(false)
{
}

void SpinAction::Initialize()
{
    // double dist;
    // getInput("spin_dist", dist);
    // double time_allowance;
    // getInput("time_allowance", time_allowance);
    // goal_.target_yaw = dist;
    // goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
    // getInput("is_recovery", is_recovery_);

    // initialized_ = true;
}

void SpinAction::OnTick()
{
    if (!initialized_) {
        Initialize();
    }

    if (is_recovery_) {
        IncrementRecoveryCount();
    }
}

BT::NodeStatus SpinAction::OnSuccess()
{
//   setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpinAction::OnAborted()
{
    // setOutput("error_code_id", result_.result->error_code);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SpinAction::OnCancelled()
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
    BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
    {
        return std::make_unique<openbot::system::navigation::behavior_tree::SpinAction>(
            name, "spin", config);
    };

    factory.registerBuilder<openbot::system::navigation::behavior_tree::SpinAction>(
        "Spin", builder);
}