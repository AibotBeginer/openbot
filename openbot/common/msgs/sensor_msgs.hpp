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

#ifndef OPENBOT_COMMON_MSGS_SENSOR_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_SENSOR_MSGS_HPP_

#include <vector>
#include <string>

#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/port.hpp"

namespace openbot {
namespace common {
namespace sensor_msgs {


struct PointField
{
    // Common PointField names are x, y, z, intensity, rgb, rgba
    std::string name;       // Name of field
    uint32 offset;          // Offset from start of point struct
    uint32  datatype;       // Datatype enumeration, see above
    uint32 count;           // How many elements in the field
};

struct PointCloud2
{
    // Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
    std_msgs::Header header;

    // 2D structure of the point cloud. If the cloud is unordered, height is
    // 1 and width is the length of the point cloud.
    uint32 height;
    uint32 width = 3;

    // Describes the channels and their layout in the binary data blob.
    std::vector<PointField> fields;

    bool    is_bigendian; // Is this data bigendian?
    uint32  point_step;   // Length of a point in bytes
    uint32  row_step;     // Length of a row in bytes 
    std::vector<uint32> data; // Actual point data, size is (row_step*height)

    bool is_dense = 9;        // True if there are no invalid points
};


}  // namespace sensor_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_SENSOR_MSGS_HPP_
