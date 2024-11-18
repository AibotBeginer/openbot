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


#ifndef OPENBOT_PLANNING_COMMON_GLOBAL_PLANNER_HPP_
#define OPENBOT_PLANNING_COMMON_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "openbot/common/macros.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/proto/nav_msgs/path.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"
#include "openbot/map/voxel_map.hpp"
#include "openbot/map/costmap.hpp"

namespace openbot {
namespace planning { 


/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class GlobalPlanner
{
public:

   /**
   *  @brief SharedPtr typedef
   */
  OPENBOT_SMART_PTR_DEFINITIONS(GlobalPlanner)

  /**
   * @brief Virtual destructor
   */
  virtual ~GlobalPlanner() {}

  /**
   * @param  name The name of this planner
   */
  virtual void Configure(std::string name) = 0;

  /**
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap
   */
  virtual void Configure(std::string name, std::shared_ptr<map::Costmap> costmap_ros) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void Cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void Activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void Deactivate() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual common::proto::nav_msgs::Path CreatePlan(
    const common::proto::geometry_msgs::PoseStamped& start,
    const common::proto::geometry_msgs::PoseStamped & goal) = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual common::nav_msgs::Path CreatePlan(
    const common::geometry_msgs::PoseStamped& start,
    const common::geometry_msgs::PoseStamped & goal) = 0;
};

}  // namespace planning 
}  // namespace openbot

#endif  // OPENBOT_PLANNING_COMMON_GLOBAL_PLANNER_HPP_
