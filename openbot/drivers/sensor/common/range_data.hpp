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

#include "openbot/drivers/sensor/common/point_cloud.hpp"
#include "openbot_bridge/common_msgs/sensor_msgs.pb.h"
#include "openbot_bridge/common_msgs/geometry_msgs.pb.h"

namespace openbot {
namespace drivers {
namespace sensor { 

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData 
{
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
};

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Converts 'range_data' to a openbot_bridge::common_msgs::Range 
openbot_bridge::common_msgs::Range ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
RangeData FromProto(const openbot_bridge::common_msgs::Range& proto);

}  // namespace sensor
}  // namespace drivers
}  // namespace openbot

