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

#include "openbot/system/navigation/behavior_tree/bt_service_node.hpp"

#include "openbot/system/navigation/proto/clear_entire_costmap.pb.h"
#include "openbot/system/navigation/proto/clear_costmap_except_region.pb.h"
#include "openbot/system/navigation/proto/clear_costmap_around_robot.pb.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps nav2_msgs::srv::ClearEntireCostmap
 */
class ClearEntireCostmapService : public BtServiceNode<openbot::navigation::ClearEntireCostmap>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ClearEntireCostmapService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ClearEntireCostmapService(
        const std::string& service_node_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void OnTick() override;
};


/**
 * @brief A nav2_behavior_tree::BtServiceNode class that
 * wraps nav2_msgs::srv::ClearCostmapExceptRegion
 */
class ClearCostmapExceptRegionService
  : public BtServiceNode<openbot::navigation::ClearCostmapExceptRegion>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ClearCostmapExceptRegionService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ClearCostmapExceptRegionService(
        const std::string& service_node_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void OnTick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<double>(
                "reset_distance", 1, 
                "Distance from the robot above which obstacles are cleared")
        });
    }
};


/**
 * @brief A nav2_behavior_tree::BtServiceNode class that
 * wraps nav2_msgs::srv::ClearCostmapAroundRobot
 */
class ClearCostmapAroundRobotService : public BtServiceNode<openbot::navigation::ClearCostmapAroundRobot>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ClearCostmapAroundRobotService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ClearCostmapAroundRobotService(
        const std::string& service_node_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void OnTick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<double>(
                "reset_distance", 1,
                "Distance from the robot under which obstacles are cleared")
        });
    }
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot