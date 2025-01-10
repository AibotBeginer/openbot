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

#include "openbot/common/io/msgs.hpp"
#include "behaviortree_cpp/action_node.h"

                                
namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

// class RemoveInCollisionGoals : public BtServiceNode<nav2_msgs::srv::GetCosts>
// {
// public:
//     typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

//     /**
//      * @brief A constructor for nav2_behavior_tree::RemoveInCollisionGoals
//      * @param service_node_name Service name this node creates a client for
//      * @param conf BT node configuration
//      */
//     RemoveInCollisionGoals(
//         const std::string & service_node_name,
//         const BT::NodeConfiguration & conf);

//     /**
//      * @brief The main override required by a BT service
//      * @return BT::NodeStatus Status of tick execution
//      */
//     void on_tick() override;

//     BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
//     override;

//     static BT::PortsList providedPorts()
//     {
//         return providedBasicPorts(
//         {
//             BT::InputPort<Goals>("input_goals", "Original goals to remove from"),
//             BT::InputPort<double>("cost_threshold", 254.0, "Cost threshold for considering a goal in collision"),
//             BT::InputPort<bool>("use_footprint", true, "Whether to use footprint cost"),
//             BT::InputPort<bool>("consider_unknown_as_obstacle", false, "Whether to consider unknown cost as obstacle"),
//             BT::OutputPort<Goals>("output_goals", "Goals with in-collision goals removed"),
//         });
//     }

// private:
//   bool use_footprint_;
//   bool consider_unknown_as_obstacle_;
//   double cost_threshold_;
//   Goals input_goals_;
// };

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot