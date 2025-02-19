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

#include "openbot/system/navigation/behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string& xml_tag_name,
  const std::string& action_name,
  const BT::NodeConfiguration& conf)
: BtActionNode<openbot::navigation::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathThroughPosesAction::OnTick()
{
    // getInput("goals", request_.goals());
    // getInput("planner_id", request_.planner_id());
    // if (getInput("start", request_.start())) {
    //     request_.set_use_start(true);
    // }
}

BT::NodeStatus ComputePathThroughPosesAction::OnSuccess()
{
    // setOutput("path", response_.path());
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::OnAborted()
{
//   common::nav_msgs::Path empty_path;
//   setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::OnCancelled()
{
    // common::nav_msgs::Path empty_path;
    // setOutput("path", empty_path);
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
        return std::make_unique<openbot::system::navigation::behavior_tree::ComputePathThroughPosesAction>(
            name, "compute_path_through_poses", config);
    };

    factory.registerBuilder<openbot::system::navigation::behavior_tree::ComputePathThroughPosesAction>(
        "ComputePathThroughPoses", builder);
}