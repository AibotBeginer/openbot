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

#include "openbot/system/navigation/behavior_tree/plugins/action/truncate_path_action.hpp"

#include "openbot/system/navigation/common/geometry_utils.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

TruncatePath::TruncatePath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  distance_(1.0)
{
}

inline BT::NodeStatus TruncatePath::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    getInput("distance", distance_);

    common::nav_msgs::Path input_path;

    getInput("input_path", input_path);

    // if (input_path.poses.empty()) {
    //     setOutput("output_path", input_path);
    //     return BT::NodeStatus::SUCCESS;
    // }

    // common::geometry_msgs::PoseStamped final_pose = input_path.poses.back();

    // double distance_to_goal = geometry_utils::euclidean_distance(
    //     input_path.poses.back(), final_pose);

    // while (distance_to_goal < distance_ && input_path.poses.size() > 2) {
    //     input_path.poses.pop_back();
    //     distance_to_goal = geometry_utils::euclidean_distance(
    //     input_path.poses.back(), final_pose);
    // }

    // double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
    // double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

    // double final_angle = atan2(dy, dx);

    // if (std::isnan(final_angle) || std::isinf(final_angle)) {
    //     RCLCPP_WARN(
    //     config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
    //     "Final angle is not valid while truncating path. Setting to 0.0");
    //     final_angle = 0.0;
    // }

    // input_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    //     final_angle);

    setOutput("output_path", input_path);

    return BT::NodeStatus::SUCCESS;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<openbot::system::navigation::behavior_tree::TruncatePath>("TruncatePath");
}