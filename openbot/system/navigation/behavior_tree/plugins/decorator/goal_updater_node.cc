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

#include "openbot/system/navigation/behavior_tree/plugins/decorator/goal_updater_node.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

using std::placeholders::_1;

GoalUpdater::GoalUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
    // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // std::string goal_updater_topic;
    // node_->get_parameter_or<std::string>("goal_updater_topic", goal_updater_topic, "goal_update");

    // goal_sub_ = node_->create_subscription<common::geometry_msgs::PoseStamped>(
    //     goal_updater_topic,
    //     rclcpp::SystemDefaultsQoS(),
    //     std::bind(&GoalUpdater::callback_updated_goal, this, _1),
    //     sub_option);
}

inline BT::NodeStatus GoalUpdater::tick()
{
    common::geometry_msgs::PoseStamped goal;

    getInput("input_goal", goal);

    // if (last_goal_received_.header.stamp != rclcpp::Time(0)) {
    //     auto last_goal_received_time = rclcpp::Time(last_goal_received_.header.stamp);
    //     auto goal_time = rclcpp::Time(goal.header.stamp);
    //     if (last_goal_received_time > goal_time) {
    //     goal = last_goal_received_;
    //     } else {
    //     RCLCPP_WARN(
    //         node_->get_logger(), "The timestamp of the received goal (%f) is older than the "
    //         "current goal (%f). Ignoring the received goal.",
    //         last_goal_received_time.seconds(), goal_time.seconds());
    //     }
    // }

    setOutput("output_goal", goal);
    return child_node_->executeTick();
}

void GoalUpdater::callback_updated_goal(const std::shared_ptr<common::geometry_msgs::PoseStamped> msg)
{
    last_goal_received_ = *msg;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::GoalUpdater>("GoalUpdater");
}