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

#include "openbot/system/navigation/behavior_tree/plugins/action/follow_path_action.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<openbot::navigation::FollowPath>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::OnTick()
{
    // getInput("path", request_.path());
    // getInput("controller_id", request_.controller_id());
    // getInput("goal_checker_id", request_.goal_checker_id());
}

void FollowPathAction::OnWaitForResult(
  std::shared_ptr<const openbot::navigation::FollowPath::Response >/*response*/)
{
    // Grab the new path
    common::nav_msgs::Path new_path;
    getInput("path", new_path);

    // // Check if it is not same with the current one
    // if (request_.path() != new_path) {
    //     // the action server on the next loop iteration
    //     goal_.path() = new_path;
    //     goal_updated_ = true;
    // }

    // std::string new_controller_id;
    // getInput("controller_id", new_controller_id);

    // if (request_.controller_id != new_controller_id) {
    //     request_.controller_id = new_controller_id;
    //     goal_updated_ = true;
    // }

    // std::string new_goal_checker_id;
    // getInput("goal_checker_id", new_goal_checker_id);

    // if (request_.goal_checker_id() != new_goal_checker_id) {
    //     request_.goal_checker_id() = new_goal_checker_id;
    //     goal_updated_ = true;
    // }
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
        return std::make_unique<openbot::system::navigation::behavior_tree::FollowPathAction>(
            name, "follow_path", config);
    };

    factory.registerBuilder<openbot::system::navigation::behavior_tree::FollowPathAction>(
        "FollowPath", builder);
}