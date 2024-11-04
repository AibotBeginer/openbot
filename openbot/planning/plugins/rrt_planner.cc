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
    costmap_ = costmap;
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
    // auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
    // ompl::base::RealVectorBounds bounds(3);
    // bounds.setLow(0, 0.0);
    // bounds.setHigh(0, hb(0) - lb(0));
    // bounds.setLow(1, 0.0);
    // bounds.setHigh(1, hb(1) - lb(1));
    // bounds.setLow(2, 0.0);
    // bounds.setHigh(2, hb(2) - lb(2));
    // space->setBounds(bounds);
    // auto si(std::make_shared<ompl::base::SpaceInformation>(space));sss
    // si->setStateValidityChecker([&](const ompl::base::State *state) {
    //         const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    //         const Eigen::Vector3d position(lb(0) + (*pos)[0], lb(1) + (*pos)[1], lb(2) + (*pos)[2]);
    //         return costmap_->Query(position) == 0;
    // });
    // si->setup();

    // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    // ompl::base::ScopedState<> start(space), goal(space);
    // start[0] = s(0) - lb(0);
    // start[1] = s(1) - lb(1);
    // start[2] = s(2) - lb(2);
    // goal[0] = g(0) - lb(0);
    // goal[1] = g(1) - lb(1);
    // goal[2] = g(2) - lb(2);
    // auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    // pdef->setStartAndGoalStates(start, goal);
    // pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
    // auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
    // planner->setProblemDefinition(pdef);
    // planner->setup();

    // ompl::base::PlannerStatus solved;
    // solved = planner->ompl::base::Planner::solve(timeout);
   
    // if !(solved) {
    //     LOG(WARN) << "RRT Planner use ompl rrt planner solved failed.";
    //     return path;
    // }

    // double cost = INFINITY;
    // p.clear();
    // const ompl::geometric::PathGeometric path_ = ompl::geometric::PathGeometric(
    //     dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
    // for (size_t i = 0; i < path_.getStateCount(); i++)
    // {
    //     const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    //     p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
    // }
    // cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
    // LOG(INFO) << "RRT Planner create plan cost: " << cost;

    common::nav_msgs::Path path;
    return path;
}

}  // namespace plugins
}  // namespace planning 
}  // namespace openbot