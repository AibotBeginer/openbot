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


#include "openbot/planning/planner_server.hpp"
// #include "openbot/common/proto/nav_msgs/path.pb.h"
// #include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

namespace openbot {
namespace planning { 

PlannerServer::PlannerServer()
{
    // std::string name = "rrt_planner";
    // plugins_[name] = std::make_shared<plugins::RRTPlanner>();
    
    SetRunningPlanner("a_star_planner");
    plugins_[running_planner_name()] = std::make_shared<plugins::AStarPlanner>();
}

PlannerServer::~PlannerServer()
{
}

void PlannerServer::InitMap(const map::Costmap::SharedPtr costmap)
{
    costmap_ = costmap;
    plugins_[running_planner_name()]->Configure(running_planner_name(),  costmap_);
}

common::nav_msgs::Path PlannerServer::CreatePlan(
    const common::geometry_msgs::PoseStamped& start,
    const common::geometry_msgs::PoseStamped& goal)
{
    return plugins_[running_planner_name()]->CreatePlan(start, goal);
}

void PlannerServer::SetRunningPlanner(const std::string& name)
{
    running_planner_name_ = name;
}

std::string PlannerServer::running_planner_name()
{
    return running_planner_name_;
}

}  // namespace planning 
}  // namespace openbot