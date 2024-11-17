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

#ifndef OPENBOT_PLANNING_PLUGINS_A_STAR_PLANNER_HPP_
#define OPENBOT_PLANNING_PLUGINS_A_STAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <queue>

#include <Eigen/Eigen>

#include "openbot/common/macros.hpp"
#include "openbot/planning/common/global_planner.hpp"
#include "openbot/common/proto/nav_msgs/path.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

namespace openbot {
namespace planning { 
namespace plugins { 


#define INF (1 << 30);
#define IN_OPEN_LIST   'a'
#define IN_CLOSE_LIST  'b'
#define NOT_EXPANDED   'c'

struct PathNode 
{
public:
    Eigen::Vector3d position;
    double g_cost, f_cost;
    char node_state;
    PathNode* parent;

    PathNode() 
    {
      g_cost = INF;
      f_cost = INF;
      parent = nullptr;
      node_state = NOT_EXPANDED;
    }
};
typedef PathNode* PathNodePtr;

class NodeComparator {
  public:
    bool operator()(const PathNodePtr& node1, const PathNodePtr& node2) const {
      return node1->f_cost > node2->f_cost;
    }
};

template<typename T>
struct hash_function : std::unary_function<T, size_t> 
{
    size_t operator()(T const& x) const 
    {
      size_t seed = 0;
      for (size_t i = 0; i < x.size(); ++i) {
          auto elem = *(x.data() + i);
          seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      } 
      return seed;
    } 
};

class NodeHashTable 
{
public:
    void Insert(Eigen::Vector3d pos, PathNodePtr node) 
    {
        node_table_.insert(std::make_pair(pos, node));
    }

    PathNodePtr Find(Eigen::Vector3d pos) 
    {
        auto it = node_table_.find(pos);
        if (it != node_table_.end()) {  
          return it->second;
        }
        return nullptr;
    }

    void Erase(Eigen::Vector3d pos) 
    {
        node_table_.erase(pos);
    }

    void Clear() 
    {
        node_table_.clear();
    }

private:
    // don't use Eigen::Vector3i index as key, because different position may have the same index
    std::unordered_map<Eigen::Vector3d, PathNodePtr, hash_function<Eigen::Vector3d>> node_table_;
};

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 */
class AStarPlanner : public GlobalPlanner
{
public:
  /**
   *  @brief SharedPtr
   */
  OPENBOT_SMART_PTR_DEFINITIONS(AStarPlanner)

  /**
   * @brief Virtual destructor
   */
  virtual ~AStarPlanner();

  /**
   * @param name
   */
  virtual void Configure(std::string name) override;

  /**
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap
   */
  virtual void Configure(std::string name, std::shared_ptr<map::Costmap> costmap) override;

  /**
   * @brief Method to cleanup resources.
   */
  virtual void Cleanup() override;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void Activate() override;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void Deactivate() override;

   /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual common::proto::nav_msgs::Path CreatePlan(
    const common::proto::geometry_msgs::PoseStamped& start,
    const common::proto::geometry_msgs::PoseStamped& goal) override;

  /**
     * @brief Method create the plan from a starting and ending goal.
     * @param start The starting pose of the robot
     * @param goal  The goal pose of the robot
     * @return      The sequence of poses to get from start to goal, if any
     */
    virtual common::nav_msgs::Path CreatePlan(
        const common::geometry_msgs::PoseStamped& start,
        const common::geometry_msgs::PoseStamped& goal) override;

private:
    double GetEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double GetDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    common::nav_msgs::Path RetrievePath(PathNodePtr node);

    map::VoxelMap* costmap_{nullptr};

    /* main data structure */
    std::priority_queue <PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_list_;
    NodeHashTable close_list_;
    NodeHashTable expanded_nodes_;
    std::vector<PathNodePtr> path_node_pool_;
    std::vector<Eigen::Vector3d> path_;

    /* main search parameters */
    int allocated_node_num_; // pre-allocated the nodes num
    int use_node_num_;      // number of the nodes expanded
    double lambda_;         // weight for the heuristic term
    double tie_breaker_;    // enhance the search velocity
};

}  // namespace plugins
}  // namespace planning 
}  // namespace openbot

#endif  // OPENBOT_PLANNING_PLUGINS_A_STAR_PLANNER_HPP_
