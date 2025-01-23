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

#include "openbot/system/navigation/behavior_tree/plugins/action/clear_costmap_service.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

ClearEntireCostmapService::ClearEntireCostmapService(
  const std::string& service_node_name,
  const BT::NodeConfiguration& conf)
: BtServiceNode<openbot::navigation::ClearEntireCostmap>(service_node_name, conf)
{
}

void ClearEntireCostmapService::OnTick()
{
  IncrementRecoveryCount();
}

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
  const std::string& service_node_name,
  const BT::NodeConfiguration& conf)
: BtServiceNode<openbot::navigation::ClearCostmapExceptRegion>(service_node_name, conf)
{
}

void ClearCostmapExceptRegionService::OnTick()
{
    // getInput("reset_distance", request_->reset_distance);
    IncrementRecoveryCount();
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
  const std::string& service_node_name,
  const BT::NodeConfiguration& conf)
: BtServiceNode<openbot::navigation::ClearCostmapAroundRobot>(service_node_name, conf)
{
}

void ClearCostmapAroundRobotService::OnTick()
{
    // getInput("reset_distance", request_->reset_distance);
    IncrementRecoveryCount();
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<openbot::system::navigation::behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
    factory.registerNodeType<openbot::system::navigation::behavior_tree::ClearCostmapExceptRegionService>(
        "ClearCostmapExceptRegion");
    factory.registerNodeType<openbot::system::navigation::behavior_tree::ClearCostmapAroundRobotService>(
        "ClearCostmapAroundRobot");
}