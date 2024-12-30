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
#include <vector>

#include "openbot/common/io/msgs.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::ConditionNode that returns SUCCESS when goal is
 * updated on the blackboard and FAILURE otherwise
 */
class GoalUpdatedCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalUpdatedCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalUpdatedCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    GoalUpdatedCondition() = delete;

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
        return {};
    }

private:
    common::geometry_msgs::PoseStamped goal_;
    std::vector<common::geometry_msgs::PoseStamped> goals_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot