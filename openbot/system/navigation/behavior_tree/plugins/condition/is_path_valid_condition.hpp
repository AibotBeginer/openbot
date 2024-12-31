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

#pragma once

#include <string>
#include <memory>

#include "cyber/cyber.h"

#include "behaviortree_cpp/condition_node.h"
#include "openbot/common/io/msgs.hpp"
// #include "nav2_msgs/srv/is_path_valid.hpp"


namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPathValid
 * service returns true and FAILURE otherwise
 */
class IsPathValidCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::IsPathValidCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    IsPathValidCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    IsPathValidCondition() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<common::nav_msgs::Path>("path", "Path to Check"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")
        };
    }

private:
    std::shared_ptr<::apollo::cyber::Node> node_;
    // rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
    // The timeout value while waiting for a responce from the
    // is path valid service
    std::chrono::milliseconds server_timeout_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot