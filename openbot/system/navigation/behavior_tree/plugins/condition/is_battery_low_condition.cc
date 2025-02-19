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

#include "openbot/system/navigation/behavior_tree/plugins/condition/is_battery_low_condition.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
    getInput("min_battery", min_battery_);
    getInput("battery_topic", battery_topic_);
    getInput("is_voltage", is_voltage_);
//   node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
//   battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
//     battery_topic_,
//     rclcpp::SystemDefaultsQoS(),
//     std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
//     sub_option);
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
    if (is_battery_low_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
// {
//   if (is_voltage_) {
//     is_battery_low_ = msg->voltage <= min_battery_;
//   } else {
//     is_battery_low_ = msg->percentage <= min_battery_;
//   }
// }

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}