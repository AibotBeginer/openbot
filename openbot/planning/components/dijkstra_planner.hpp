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

#ifndef OPENBOT_PLANNING_COMPONENTS_DIJKSTRA_PLANNER_HPP_
#define OPENBOT_PLANNING_COMPONENTS_DIJKSTRA_PLANNER_HPP_

#include <memory>
#include <string>

#include "openbot/common/macros.hpp"
#include "openbot/planning/global_planner.hpp"
#include "openbot/common/proto/nav_msgs/path.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

namespace openbot {
namespace planning { 
namespace components { 

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 */
class DijkstraPlanner : public GlobalPlanner
{
public:
  /**
   *  @brief SharedPtr
   */
  OPENBOT_SMART_PTR_DEFINITIONS(DijkstraPlanner)

  /**
   * @brief Virtual destructor
   */
  virtual ~DijkstraPlanner();

  /**
   * @param name
   */
  virtual void Configure(std::string name) override;

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

};

}  // namespace components
}  // namespace planning 
}  // namespace openbot

#endif  // OPENBOT_PLANNING_COMPONENTS_DIJKSTRA_PLANNER_HPP_
