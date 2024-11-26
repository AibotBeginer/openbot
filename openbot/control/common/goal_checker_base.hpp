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
#include "openbot/common/proto/nav_msgs/path.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

#include "openbot/control/motion/motion_common.hpp"
#include "openbot/common/motion_model/differential_drive_motion_model.hpp"


namespace openbot {
namespace control {

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 */
class GoalChecker
{
public:
  /**
   *  @brief SharedPtr
   */
  OPENBOT_SMART_PTR_DEFINITIONS(GoalChecker)

  /**
   * @brief Virtual destructor
   */
  virtual ~GoalChecker() {}

};

}  // namespace control
}  // namespace openbot
