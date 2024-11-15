
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

#ifndef OPENBOT_MAP_GRID_MAP_GRID_MAP_INFO_GRID_MAP_INFO_HPP_
#define OPENBOT_MAP_GRID_MAP_GRID_MAP_INFO_GRID_MAP_INFO_HPP_

#include <chrono>
#include <vector>
#include <string>

#include "openbot/common/port.hpp"
#include "openbot/common/msgs/msgs.hpp"


namespace grid_map {
namespace grid_map_msgs {

struct GridMapInfo
{
    // Header (time and frame)
    ::openbot::common::std_msgs::Header header;

    // Resolution of the grid [m/cell].
    double resolution;

    // Length in x-direction [m].
    double length_x;

    // Length in y-direction [m].
    double length_y;

    // Pose of the grid map center in the frame defined in `header` [m].
    ::openbot::common::geometry_msgs::Pose pose;
};

}  // grid_map_msgs
}  // namespace grid_map

#endif  // OPENBOT_MAP_GRID_MAP_GRID_MAP_INFO_GRID_MAP_INFO_HPP_




