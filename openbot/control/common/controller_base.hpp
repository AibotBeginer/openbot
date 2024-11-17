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

#pragma once

#include <chrono>
#include <string>

#include "openbot/common/macros.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/proto/nav_msgs/path.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

#include "openbot/control/motion/motion_common.hpp"
#include "openbot/common/motion_model/differential_drive_motion_model.hpp"

#define CONTROLLER_COMMON_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default; \
  Class(Class &&) = default; \
  Class & operator=(const Class &) = default; \
  Class & operator=(Class &&) = default; \
  ~Class() = default;

namespace openbot {
namespace control {

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 */
class Controller
{
public:
  /**
   *  @brief SharedPtr
   */
  OPENBOT_SMART_PTR_DEFINITIONS(Controller)

  /**
   * @brief Virtual destructor
   */
  virtual ~Controller() {}

  /**
   * @param 
   */
  virtual void Configure() = 0;

  /**
   * @brief Method to cleanup resources.
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
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   */
  virtual void SetPlan(const common::nav_msgs::Path& path) = 0;

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  virtual common::proto::geometry_msgs::TwistStamped ComputeVelocityCommands(
    const common::proto::geometry_msgs::PoseStamped& pose,
    const common::proto::geometry_msgs::Twist& velocity) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  virtual void SetSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace control
}  // namespace openbot
