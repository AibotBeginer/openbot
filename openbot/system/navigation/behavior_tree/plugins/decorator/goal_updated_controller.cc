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


#include "openbot/system/navigation/behavior_tree/plugins/decorator/goal_updated_controller.hpp"

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

GoalUpdatedController::GoalUpdatedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::NodeStatus GoalUpdatedController::tick()
{
    if (status() == BT::NodeStatus::IDLE) {
        // Reset since we're starting a new iteration of
        // the goal updated controller (moving from IDLE to RUNNING)

        config().blackboard->get<std::vector<common::geometry_msgs::PoseStamped>>("goals", goals_);
        config().blackboard->get<common::geometry_msgs::PoseStamped>("goal", goal_);

        goal_was_updated_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    // std::vector<common::geometry_msgs::PoseStamped> current_goals;
    // config().blackboard->get<std::vector<common::geometry_msgs::PoseStamped>>("goals", current_goals);
    // common::geometry_msgs::PoseStamped current_goal;
    // config().blackboard->get<common::geometry_msgs::PoseStamped>("goal", current_goal);

    // if (goal_ != current_goal || goals_ != current_goals) {
    //     goal_ = current_goal;
    //     goals_ = current_goals;
    //     goal_was_updated_ = true;
    // }

    // // The child gets ticked the first time through and any time the goal has
    // // changed or preempted. In addition, once the child begins to run, it is ticked each time
    // // 'til completion
    // if ((child_node_->status() == BT::NodeStatus::RUNNING) || goal_was_updated_) 
    // {
    //     goal_was_updated_ = false;
    //     const BT::NodeStatus child_state = child_node_->executeTick();

    //     switch (child_state) {
    //     case BT::NodeStatus::RUNNING:
    //         return BT::NodeStatus::RUNNING;

    //     case BT::NodeStatus::SUCCESS:
    //         return BT::NodeStatus::SUCCESS;

    //     case BT::NodeStatus::FAILURE:
    //     default:
    //         return BT::NodeStatus::FAILURE;
    //     }
    // }

    return status();
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::GoalUpdatedController>("GoalUpdatedController");
}