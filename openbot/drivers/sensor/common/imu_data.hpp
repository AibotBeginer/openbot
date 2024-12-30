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

#include "Eigen/Core"

#include "openbot/common/utils/time.hpp"
#include "openbot_bridge/common_msgs/builtin_interfaces.pb.h"
#include "openbot_bridge/common_msgs/sensor_msgs.pb.h"

namespace openbot {
namespace drivers {
namespace sensor { 

struct ImuData 
{
    common::Time time;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

// Converts 'imu_data' to a proto::ImuData.
openbot_bridge::common_msgs::Imu ToProto(const ImuData& imu_data);

// Converts 'proto' to an ImuData.
ImuData FromProto(const openbot_bridge::common_msgs::Imu& proto);

}  // namespace sensor
}  // namespace drivers
}  // namespace openbot