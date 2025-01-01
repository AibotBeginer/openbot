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

#include "openbot/common/utils/time.hpp"
#include "openbot/common/transform/rigid_transform.hpp"
#include "openbot_bridge/common_msgs/builtin_interfaces.pb.h"
#include "openbot_bridge/common_msgs/sensor_msgs.pb.h"
#include "openbot_bridge/common_msgs/geometry_msgs.pb.h"

namespace openbot {
namespace drivers {
namespace sensor { 

struct OdometryData 
{
  common::Time time;
  common::transform::Rigid3d pose;
};

// Converts 'odometry_data' to a proto::OdometryData.
::openbot_bridge::common_msgs::Transform ToProto(const OdometryData& odometry_data);

// Converts 'proto' to an OdometryData.
OdometryData FromProto(const ::openbot_bridge::common_msgs::Transform& proto);

}  // namespace sensor
}  // namespace drivers
}  // namespace openbot