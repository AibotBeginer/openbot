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
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h> // Include RRT# header
#include <ompl/geometric/planners/informedtrees/BITstar.h>


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


// Configures the RRTPlanner with a name and a shared pointer to a Costmap
void RRTPlanner::Configure(std::string name, std::shared_ptr<map::Costmap> costmap)
{
    // costmap_ = dynamic_cast<VoxelMap::SharedPtr>(costmap);
    costmap_ = dynamic_cast<map::VoxelMap*>(costmap.get());

    LOG(INFO) << "Cost Map Size: " << costmap_->GetSize().transpose();
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

common::nav_msgs::Path RRTPlanner::CreatePlan(
        const common::geometry_msgs::PoseStamped& start,
        const common::geometry_msgs::PoseStamped& goal,
        const double timeout)
{
    // // Define the 3D state space for the planner (x, y, z)
    // auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
    // ompl::base::RealVectorBounds bounds(3);

    // // Get bounds of the costmap
    // auto lb = costmap_->GetOrigin();  // Lower bound (origin)
    // auto hb = costmap_->GetCorner();  // Upper bound (corner)

    // // TODO: why not bounds.setLow(0, lb(0));
    // bounds.setLow(0, 0.0);
    // bounds.setHigh(0, hb(0) - lb(0));
    // bounds.setLow(1, 0.0);
    // bounds.setHigh(1, hb(1) - lb(1));
    // bounds.setLow(2, 0.0);
    // bounds.setHigh(2, hb(2) - lb(2));
    // space->setBounds(bounds);


    // auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    // si->setStateValidityChecker([&](const ompl::base::State *state) {
    //         const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    //         const Eigen::Vector3d position(lb(0) + (*pos)[0], lb(1) + (*pos)[1], lb(2) + (*pos)[2]);
    //         return costmap_->Query(position) == 0; // occupied
    // });
    // si->setup();

    // // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    // ompl::base::ScopedState<> start_pose(space), goal_pose(space);
    // start_pose[0] = start.pose.position.x - lb(0);
    // start_pose[1] = start.pose.position.y - lb(1);
    // start_pose[2] = start.pose.position.z - lb(2);
    // goal_pose[0] = goal.pose.position.x - lb(0);
    // goal_pose[1] = goal.pose.position.y - lb(1);
    // goal_pose[2] = goal.pose.position.z - lb(2);

    // LOG(INFO) << "Start Pose: ["
    //       << "x: " << start.pose.position.x << ", "
    //       << "y: " << start.pose.position.y << ", "
    //       << "z: " << start.pose.position.z << "] "
    //       << "Goal Pose: ["
    //       << "x: " << goal.pose.position.x << ", "
    //       << "y: " << goal.pose.position.y << ", "
    //       << "z: " << goal.pose.position.z << "]";

    // LOG(INFO) << "Bounds: X[" << bounds.low[0] << ", " << bounds.high[0] << "], "
    //       << "Y[" << bounds.low[1] << ", " << bounds.high[1] << "], "
    //       << "Z[" << bounds.low[2] << ", " << bounds.high[2] << "]";
    // LOG(INFO) << "Transformed Start Pose: [" << start_pose[0] << ", " << start_pose[1] << ", " << start_pose[2] << "]";
    // LOG(INFO) << "Transformed Goal Pose: [" << goal_pose[0] << ", " << goal_pose[1] << ", " << goal_pose[2] << "]";
  
    //  // Initialize random seed
    // std::srand(std::time(nullptr));

    // // Create problem definition
    // auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
    // pdef->setStartAndGoalStates(start_pose, goal_pose);
    // pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));

    // // Randomly select algorithm
    // int algorithm_choice = 1; // std::rand() % 3;  // Generate a random number between 0 and 2
    // std::string algorithm_name;
    // ompl::base::PlannerPtr planner;

    // switch (algorithm_choice)
    // {
    // case 0:
    //     planner = std::make_shared<ompl::geometric::RRTConnect>(si);
    //     algorithm_name = "RRTConnect";
    //     break;
    // case 1:
    //     planner = std::make_shared<ompl::geometric::InformedRRTstar>(si);
    //     algorithm_name = "InformedRRTStar";
    //     break;
    // case 2:
    //     planner = std::make_shared<ompl::geometric::RRTsharp>(si); // Use RRTsharp
    //     algorithm_name = "RRT#";
    //     break;
    // }

    // // Log the selected algorithm
    // LOG(INFO) << "Selected Algorithm: " << algorithm_name;
    // LOG(INFO) << "Timeout: " << timeout << " seconds";

    // // Configure the planner
    // double step_size = 0.2;  // Adjust the step size as needed
    // if (algorithm_name == "RRTConnect")
    // {
    //     std::static_pointer_cast<ompl::geometric::RRTConnect>(planner)->setRange(step_size);
    // }
    // else if (algorithm_name == "InformedRRTStar" || algorithm_name == "RRT#")
    // {
    //     std::static_pointer_cast<ompl::geometric::InformedRRTstar>(planner)->setRange(step_size);
    // }

    // planner->setProblemDefinition(pdef);
    // planner->setup();

    // auto start_time = std::chrono::high_resolution_clock::now();
    // ompl::base::PlannerStatus solved;
    // solved = planner->ompl::base::Planner::solve(timeout);
    // common::nav_msgs::Path path;

    // auto now = std::chrono::high_resolution_clock::now();
    // path.header.stamp = common::builtin_interfaces::Time::now();
    // path.header.frame_id = "map";

    // if (solved)
    // {
    //     LOG(INFO) << "Planning succeeded using " << algorithm_name;
    //     auto path = pdef->getSolutionPath();
    //     path->print(std::cout);
    // }
    // else
    // {
    //     LOG(INFO) << "Planning failed using " << algorithm_name;
    //     return path;
    // }

    // double cost = INFINITY;
    // path.poses.clear();
    // const ompl::geometric::PathGeometric path_ = ompl::geometric::PathGeometric(
    //     dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
    // for (size_t i = 0; i < path_.getStateCount(); i++)
    // {
    //     const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    //     common::geometry_msgs::PoseStamped pose;
    //     pose.pose.position.x = lb(0) + state[0];
    //     pose.pose.position.y = lb(1) + state[1];
    //     pose.pose.position.z = lb(2) + state[2];
    //     path.poses.emplace_back(pose);
    // }
    // // cost = the shortest path length for PathLengthOptimizationObjective
    // cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
    // LOG(INFO) << "RRT Planner path size : " << path.poses.size() << ", create plan cost: " << cost;

    // std::chrono::duration<double> duration = now - start_time;
    // LOG(INFO) << "Time taken to solve the problem: " << duration.count() << " seconds. OMPL Timeout: " << timeout << "s" ;

    common::nav_msgs::Path path;
    return path;
}

}  // namespace plugins
}  // namespace planning 
}  // namespace openbot