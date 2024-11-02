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


#include "openbot/planning/plugins/a_star_palnner.hpp"

namespace openbot {
namespace planning { 
namespace plugins { 

AStarPlanner::~AStarPlanner()
{

}

void AStarPlanner::Configure(std::string name)
{

}

void AStarPlanner::Configure(std::string name, std::shared_ptr<map::Costmap> costmap)
{

}

void AStarPlanner::Cleanup() 
{
  
}

void AStarPlanner::Activate() 
{

}

void AStarPlanner::Deactivate() 
{

}

common::proto::nav_msgs::Path AStarPlanner::CreatePlan(
    const common::proto::geometry_msgs::PoseStamped& start,
    const common::proto::geometry_msgs::PoseStamped& goal)
{
    common::proto::nav_msgs::Path path;
    return path;
}

}  // namespace plugins
}  // namespace planning 
}  // namespace openbot