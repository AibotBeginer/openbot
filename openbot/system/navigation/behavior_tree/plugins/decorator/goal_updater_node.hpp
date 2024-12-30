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

#include "openbot/common/io/msgs.hpp"
#include "behaviortree_cpp/decorator_node.h"


namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class GoalUpdater : public BT::DecoratorNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalUpdater
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalUpdater(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<common::geometry_msgs::PoseStamped>("input_goal", "Original Goal"),
            BT::OutputPort<common::geometry_msgs::PoseStamped>("output_goal", "Received Goal by subscription"),
        };
    }

private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Callback function for goal update topic
     * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
     */
    void callback_updated_goal(const std::shared_ptr<common::geometry_msgs::PoseStamped> msg);

    // rclcpp::Subscription<common::geometry_msgs::PoseStamped::SharedPtr goal_reader_;

    common::geometry_msgs::PoseStamped last_goal_received_;

    std::shared_ptr<::apollo::cyber::Node> node_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot
