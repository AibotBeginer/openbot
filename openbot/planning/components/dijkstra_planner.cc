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


#include "openbot/planning/components/dijkstra_planner.hpp"

namespace openbot {
namespace planning { 
namespace components { 

DijkstraPlanner::~DijkstraPlanner()
{

}

void DijkstraPlanner::Configure(std::string name)
{

}

void DijkstraPlanner::Cleanup() 
{
  
}

void DijkstraPlanner::Activate() 
{

}


void DijkstraPlanner::Deactivate() 
{

}

common::proto::nav_msgs::Path DijkstraPlanner::CreatePlan(
    const common::proto::geometry_msgs::PoseStamped& start,
    const common::proto::geometry_msgs::PoseStamped& goal)
{
    common::proto::nav_msgs::Path path;
    return path;
}

}  // namespace components
}  // namespace planning 
}  // namespace openbot