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
#include <atomic>
#include <deque>

#include "cyber/cyber.h"

#include "behaviortree_cpp/condition_node.h"
#include "openbot/common/io/msgs.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::ConditionNode that tracks robot odometry and returns SUCCESS
 * if robot is stuck somewhere and FAILURE otherwise
 */
class IsStuckCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::IsStuckCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    IsStuckCondition(
        const std::string& condition_name,
        const BT::NodeConfiguration& conf);

    IsStuckCondition() = delete;

    /**
     * @brief A destructor for nav2_behavior_tree::IsStuckCondition
     */
    ~IsStuckCondition() override;

    /**
     * @brief Callback function for odom topic
     * @param msg Shared pointer to nav_msgs::msg::Odometry::SharedPtr message
     */
    void OnOdomReceived(const std::shared_ptr<common::nav_msgs::Odometry> msg);

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Function to log status when robot is stuck/free
     */
    void LogStuck(const std::string& msg) const;

    /**
     * @brief Function to approximate acceleration from the odom history
     */
    void UpdateStates();

    /**
     * @brief Detect if robot bumped into something by checking for abnormal deceleration
     * @return bool true if robot is stuck, false otherwise
     */
    bool IsStuck();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {return {};}

private:
    // The node that will be used for any cyberRT operations
    std::shared_ptr<::apollo::cyber::Node> node_;
    std::atomic<bool> is_stuck_;

    // Listen to odometry
    // rclcpp::Subscription<common::nav_msgs::Odometry>::SharedPtr odom_sub_;
    // Store history of odometry measurements
    std::deque<common::nav_msgs::Odometry> odom_history_;
    std::deque<common::nav_msgs::Odometry>::size_type odom_history_size_;

    // Calculated states
    double current_accel_;

    // Robot specific paramters
    double brake_accel_limit_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot