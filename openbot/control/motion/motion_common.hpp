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

#include <algorithm>
#include <cmath>

#include "openbot/common/msgs/msgs.hpp"

namespace openbot {
namespace control {
namespace motion {

// Use same representation as message type
// using Real = decltype(autoware_auto_msgs::msg::TrajectoryPoint::x);
// using Command = openbot::common::geometry_msgs::TwistStamped;
// using Diagnostic = openbot::common::ControlDiagnostic;
// using State = autoware_auto_msgs::msg::VehicleKinematicState;
// using Trajectory = autoware_auto_msgs::msg::Trajectory;
// using Heading = decltype(decltype(State::state)::heading);
// using Index = size_t;
// using Point = openbot::common::geometry_msgs::Point;

}  // namespace motion
}  // namespace control
}  // namespace openbot

