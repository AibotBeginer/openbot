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
    : default_ids_{"GridBased"},
      default_types_{"NavfnPlanner"}
{
    // std::string name = "rrt_planner";
    // planners_[name] = std::make_shared<plugins::RRTPlanner>();
    
    // if (planner_ids_ == default_ids_) {
    //     for (size_t i = 0; i < default_ids_.size(); ++i) {
    //     }
    // }

    SetRunningPlanner("a_star_planner");
    planners_[running_planner_name()] = std::make_shared<plugins::AStarPlanner>();
}

PlannerServer::~PlannerServer()
{
    planners_.clear();
}

common::nav_msgs::Path PlannerServer::GetPlan(
    const common::geometry_msgs::PoseStamped& start,
    const common::geometry_msgs::PoseStamped& goal,
    const std::string & planner_id)
{
    common::nav_msgs::Path plan;
    return plan;
}

void PlannerServer::InitMap(const map::Costmap::SharedPtr costmap)
{
    costmap_ = costmap;
    planners_[running_planner_name()]->Configure(running_planner_name(),  costmap_);
}

common::nav_msgs::Path PlannerServer::CreatePlan(
    const common::geometry_msgs::PoseStamped& start,
    const common::geometry_msgs::PoseStamped& goal)
{
    return planners_[running_planner_name()]->CreatePlan(start, goal);
}

void PlannerServer::SetRunningPlanner(const std::string& name)
{
    running_planner_name_ = name;
}

std::string PlannerServer::running_planner_name()
{
    return running_planner_name_;
}

void PlannerServer::ComputePlan()
{

}

}  // namespace planning 
}  // namespace openbot