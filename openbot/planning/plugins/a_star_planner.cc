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


#include "openbot/planning/plugins/a_star_planner.hpp"

#include "glog/logging.h"

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
    costmap_ = dynamic_cast<map::VoxelMap*>(costmap.get());
    lambda_ = 1.0;
    allocated_node_num_ = 100000;

    // allocate memory for path_node_pool_
    path_node_pool_.resize(allocated_node_num_); 
    for (int i = 0; i < allocated_node_num_; i++)
    {
      path_node_pool_[i] = new PathNode();
    }
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
    const common::proto::geometry_msgs::PoseStamped& goal, const double timeout)
{
    common::proto::nav_msgs::Path path;
    return path;
}

common::nav_msgs::Path AStarPlanner::CreatePlan(
        const common::geometry_msgs::PoseStamped& start,
        const common::geometry_msgs::PoseStamped& goal, const double timeout)
{
    common::nav_msgs::Path path;
    Eigen::Vector3d start_point, goal_point;
    start_point << start.pose.position.x, start.pose.position.y, start.pose.position.z;
    goal_point  << goal.pose.position.x, goal.pose.position.y, goal.pose.position.z;

    if (!costmap_->Query(start_point)) {
        LOG(ERROR) << "start point is out of map.";
        return path;
    }

    if (!costmap_->Query(goal_point)) {
        LOG(ERROR) << "end point is out of map.";
        return path;
    }

    double resolution = costmap_->resolution();
    // push start_node into open_list
    PathNodePtr start_node = path_node_pool_[use_node_num_]; // bug fixed
    start_node->g_cost = 0.0;
    start_node->position = start_point;
    start_node->parent = nullptr;
    start_node->f_cost = lambda_ * GetDiagonalHeu(start_point, goal_point);
    start_node->node_state = IN_OPEN_LIST;

    open_list_.push(start_node);
    expanded_nodes_.Insert(start_node->position, start_node);
    use_node_num_ += 1;

    while (!open_list_.empty()) 
    {
        // pop node from open_list with lowest f_cost 
        // and add it into close_list
        PathNodePtr current_node = open_list_.top();
        open_list_.pop();
        current_node->node_state = IN_CLOSE_LIST;
        close_list_.Insert(current_node->position, current_node);

        // if current_node is end_pt, retrieve path and return success
        if (abs(current_node->position(0)- goal_point(0)) < resolution &&
            abs(current_node->position(1)- goal_point(1)) < resolution &&
            abs(current_node->position(2)- goal_point(2)) < resolution)
        {
            PathNodePtr end_node = current_node;
            path = RetrievePath(end_node);
        }


        for (double x = -resolution; x <= resolution; x += resolution)
        {
            for (double y = -resolution; y <= resolution; y += resolution)
            {
                for (double z = -resolution; z <= resolution; z += resolution) 
                {
                    Eigen::Vector3d neighbor_pos = current_node->position + Eigen::Vector3d(x, y, z);

                    // if neighbor is obstacle, skip it
                    if (!costmap_->Query(neighbor_pos)) {
                        continue;
                    }
                
                    // if neighbor is already in close_list, skip it
                    if (close_list_.Find(neighbor_pos) != nullptr) {
                        continue;
                    }
                    
                    double delta_pos = Eigen::Vector3d(x, y, z).norm();
                    double tmp_g_cost = current_node->g_cost + delta_pos;
                    PathNodePtr tmp_node = expanded_nodes_.Find(neighbor_pos);

                    if (tmp_node == nullptr) {
                        PathNodePtr neighbor_node = path_node_pool_[use_node_num_];
                        use_node_num_+= 1;
                        neighbor_node->g_cost = tmp_g_cost;
                        neighbor_node->position = neighbor_pos;
                        neighbor_node->parent = current_node;
                        neighbor_node->f_cost = neighbor_node->g_cost + lambda_ * GetDiagonalHeu(neighbor_pos, goal_point);
                        neighbor_node->node_state = IN_OPEN_LIST;
                        open_list_.push(neighbor_node);
                        expanded_nodes_.Insert(neighbor_node->position, neighbor_node);
                        if (use_node_num_ >= allocated_node_num_) {
                            LOG(ERROR) << "allocated_node_num is too small.";
                            return path; 
                        }
                    }
                    else if (tmp_g_cost < tmp_node->g_cost)
                    {
                        tmp_node->g_cost = tmp_g_cost;
                        tmp_node->parent = current_node;
                        tmp_node->f_cost = tmp_node->g_cost + lambda_ * GetDiagonalHeu(tmp_node->position, goal_point);
                    }
                }
            }
        }
    }

    return path;
}

double AStarPlanner::GetEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
    return tie_breaker_ * (x1 - x2).norm();
}

double AStarPlanner::GetDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
    double dx = std::abs(x1(0)- x2(0));
    double dy = std::abs(x1(1)- x2(1));
    double dz = std::abs(x1(2)- x2(2));
    double min_xyz=std::min({dx,dy,dz});
    double h = dx + dy + dz + (std::sqrt(3) - 3) * min_xyz;
    return tie_breaker_ * h;
}

common::nav_msgs::Path AStarPlanner::RetrievePath(PathNodePtr end_node)
{
    std::vector<Eigen::Vector3d> path;
    PathNodePtr current_node = end_node;
    while (current_node->parent != nullptr) {
      path.push_back(current_node->position);
      current_node = current_node->parent;
    }
    path.push_back(current_node->position);
    std::reverse(path.begin(), path.end());

    common::nav_msgs::Path plan;

    plan.header.stamp = common::builtin_interfaces::Time::now();
    plan.header.frame_id = "map";

    for (auto point : path) {
        common::geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        plan.poses.emplace_back(pose);
    }
    return plan;
}

}  // namespace plugins
}  // namespace planning 
}  // namespace openbot