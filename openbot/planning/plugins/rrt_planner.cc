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


#include "openbot/planning/plugins/rrt_planner.hpp"

#include <deque>
#include <memory>
#include <Eigen/Eigen>

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include "glog/logging.h"

namespace openbot {
namespace planning { 
namespace plugins { 

RRTPlanner::~RRTPlanner()
{

}

void RRTPlanner::Configure(std::string name)
{

}

void RRTPlanner::Configure(std::string name, std::shared_ptr<map::Costmap> costmap)
{
    // costmap_ = dynamic_cast<VoxelMap::SharedPtr>(costmap);
    costmap_ = dynamic_cast<map::VoxelMap*>(costmap.get());
}

void RRTPlanner::Cleanup() 
{
  
}

void RRTPlanner::Activate() 
{

}

void RRTPlanner::Deactivate() 
{

}

common::proto::nav_msgs::Path RRTPlanner::CreatePlan(
    const common::proto::geometry_msgs::PoseStamped& start,
    const common::proto::geometry_msgs::PoseStamped& goal)
{
    common::proto::nav_msgs::Path path;
    return path;
}

common::nav_msgs::Path RRTPlanner::CreatePlan(
        const common::geometry_msgs::PoseStamped& start,
        const common::geometry_msgs::PoseStamped& goal)
{
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
    ompl::base::RealVectorBounds bounds(3);

    auto lb = costmap_->GetOrigin();
    auto hb = costmap_->GetCorner();
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, hb(0) - lb(0));
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, hb(1) - lb(1));
    bounds.setLow(2, 0.0);
    bounds.setHigh(2, hb(2) - lb(2));
    space->setBounds(bounds);
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    si->setStateValidityChecker([&](const ompl::base::State *state) {
            const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
            const Eigen::Vector3d position(lb(0) + (*pos)[0], lb(1) + (*pos)[1], lb(2) + (*pos)[2]);
            return costmap_->Query(position) == 0 && position(2) < 0.5;
    });
    si->setup();

    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    ompl::base::ScopedState<> start_pose(space), goal_pose(space);
    start_pose[0] = start.pose.position.x - lb(0);
    start_pose[1] = start.pose.position.y - lb(1);
    start_pose[2] = start.pose.position.z - lb(2);
    goal_pose[0] = goal.pose.position.x - lb(0);
    goal_pose[1] = goal.pose.position.x - lb(1);
    goal_pose[2] = goal.pose.position.x - lb(2);
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start_pose, goal_pose);
    pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    const double timeout = 0.01;
    ompl::base::PlannerStatus solved;
    solved = planner->ompl::base::Planner::solve(timeout);
    common::nav_msgs::Path path;

    auto now = std::chrono::high_resolution_clock::now();
    path.header.stamp.sec = std::chrono::time_point_cast<std::chrono::seconds>(now).time_since_epoch().count();
    path.header.stamp.nanosec =  std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();
    path.header.frame_id = "map";

    if (!solved) {
        LOG(WARNING) << "RRT Planner use ompl rrt planner solved failed.";
        return path;
    }

    double cost = INFINITY;
    path.poses.clear();
    const ompl::geometric::PathGeometric path_ = ompl::geometric::PathGeometric(
        dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
    for (size_t i = 0; i < path_.getStateCount(); i++)
    {
        const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        common::geometry_msgs::PoseStamped pose;
        pose.pose.position.x = lb(0) + state[0];
        pose.pose.position.y = lb(1) + state[1];
        pose.pose.position.z = lb(2) + state[2];
        path.poses.emplace_back(pose);
    }
    cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
    LOG(INFO) << "RRT Planner path size : " << path.poses.size() << ", create plan cost: " << cost;
    return path;
}

}  // namespace plugins
}  // namespace planning 
}  // namespace openbot