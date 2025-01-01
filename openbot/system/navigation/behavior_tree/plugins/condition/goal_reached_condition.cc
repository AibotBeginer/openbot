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

#include "openbot/system/navigation/behavior_tree/plugins/condition/goal_reached_condition.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

GoalReachedCondition::GoalReachedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);
}

GoalReachedCondition::~GoalReachedCondition()
{
    cleanup();
}

BT::NodeStatus GoalReachedCondition::tick()
{
    if (!initialized_) {
        Initialize();
    }

    if (IsGoalReached()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void GoalReachedCondition::Initialize()
{
    node_ = config().blackboard->get<std::shared_ptr<::apollo::cyber::Node>>("node");
    tf_ = config().blackboard->get<std::shared_ptr<::openbot::transform::Buffer>>("tf_buffer");

    // node_->get_parameter("transform_tolerance", transform_tolerance_);

    initialized_ = true;
}

bool GoalReachedCondition::IsGoalReached()
{
    common::geometry_msgs::PoseStamped current_pose;

    // if (!nav2_util::getCurrentPose(
    //     current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    // {
    //     RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    //     return false;
    // }

    // common::geometry_msgs::PoseStamped goal;
    // getInput("goal", goal);
    // double dx = goal.pose.position.x - current_pose.pose.position.x;
    // double dy = goal.pose.position.y - current_pose.pose.position.y;

    // return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
    return true;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::GoalReachedCondition>("GoalReached");
}