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
#include "openbot/transform/buffer.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class GoalReachedCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalReachedCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalReachedCondition(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    GoalReachedCondition() = delete;

    /**
     * @brief A destructor for nav2_behavior_tree::GoalReachedCondition
     */
    ~GoalReachedCondition() override;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Function to read parameters and initialize class variables
     */
    void Initialize();

    /**
     * @brief Checks if the current robot pose lies within a given distance from the goal
     * @return bool true when goal is reached, false otherwise
     */
    bool IsGoalReached();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<common::geometry_msgs::PoseStamped>("goal", "Destination"),
            BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
            BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
        };
    }

protected:
    /**
     * @brief Cleanup function
     */
    void cleanup() {}

private:
    std::shared_ptr<::apollo::cyber::Node> node_;
    std::shared_ptr<::openbot::transform::Buffer> tf_;

    bool initialized_;
    double goal_reached_tol_;
    std::string global_frame_;
    std::string robot_base_frame_;
    double transform_tolerance_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot