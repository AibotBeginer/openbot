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

#include "openbot/system/navigation/behavior_tree/plugins/condition/transform_available_condition.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

TransformAvailableCondition::TransformAvailableCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  was_found_(false)
{
  node_ = config().blackboard->get<std::shared_ptr<::apollo::cyber::Node>>("node");
  tf_ = config().blackboard->get<std::shared_ptr<::openbot::transform::Buffer>>("tf_buffer");

  getInput("child", child_frame_);
  getInput("parent", parent_frame_);

//   if (child_frame_.empty() || parent_frame_.empty()) {
//     RCLCPP_FATAL(
//       node_->get_logger(), "Child frame (%s) or parent frame (%s) were empty.",
//       child_frame_.c_str(), parent_frame_.c_str());
//     exit(-1);
//   }

//   RCLCPP_DEBUG(node_->get_logger(), "Initialized an TransformAvailableCondition BT node");
}

TransformAvailableCondition::~TransformAvailableCondition()
{
//   RCLCPP_DEBUG(node_->get_logger(), "Shutting down TransformAvailableCondition BT node");
}

BT::NodeStatus TransformAvailableCondition::tick()
{
    if (was_found_) {
        return BT::NodeStatus::SUCCESS;
    }

    // std::string tf_error;
    // bool found = tf_->canTransform(
    //     child_frame_, parent_frame_, tf2::TimePointZero, &tf_error);

    // if (found) {
    //     was_found_ = true;
    //     return BT::NodeStatus::SUCCESS;
    // }

    // RCLCPP_INFO(
    //     node_->get_logger(), "Transform from %s to %s was not found, tf error: %s",
    //     child_frame_.c_str(), parent_frame_.c_str(), tf_error.c_str());

    return BT::NodeStatus::FAILURE;

}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::TransformAvailableCondition>("TransformAvailable");
}