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

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"

#include "openbot/common/io/msgs.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::DecoratorNode that ticks its child if the goal was updated
 */
class GoalUpdatedController : public BT::DecoratorNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalUpdatedController
     * @param name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalUpdatedController(
        const std::string & name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    bool goal_was_updated_;
    common::geometry_msgs::PoseStamped goal_;
    std::vector<common::geometry_msgs::PoseStamped> goals_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot
