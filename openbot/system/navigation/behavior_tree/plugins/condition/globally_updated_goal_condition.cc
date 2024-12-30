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

#include "openbot/system/navigation/behavior_tree/plugins/condition/globally_updated_goal_condition.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

GloballyUpdatedGoalCondition::GloballyUpdatedGoalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  first_time(true)
{
    node_ = config().blackboard->get<std::shared_ptr<::apollo::cyber::Node>>("node");
}

BT::NodeStatus GloballyUpdatedGoalCondition::tick()
{
    if (first_time) {
        first_time = false;
        config().blackboard->get<std::vector<common::geometry_msgs::PoseStamped>>("goals", goals_);
        config().blackboard->get<common::geometry_msgs::PoseStamped>("goal", goal_);
        return BT::NodeStatus::SUCCESS;
    }

    std::vector<common::geometry_msgs::PoseStamped> current_goals;
    config().blackboard->get<std::vector<common::geometry_msgs::PoseStamped>>("goals", current_goals);
    common::geometry_msgs::PoseStamped current_goal;
    config().blackboard->get<common::geometry_msgs::PoseStamped>("goal", current_goal);

    // if (goal_ != current_goal || goals_ != current_goals) {
    //     goal_ = current_goal;
    //     goals_ = current_goals;
    //     return BT::NodeStatus::SUCCESS;
    // }

    return BT::NodeStatus::FAILURE;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::GloballyUpdatedGoalCondition>("GlobalUpdatedGoal");
}