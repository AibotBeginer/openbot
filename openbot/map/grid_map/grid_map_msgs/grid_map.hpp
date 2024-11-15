
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

#ifndef OPENBOT_MAP_GRID_MAP_GRID_MAP_INFO_GRID_MAP_HPP_
#define OPENBOT_MAP_GRID_MAP_GRID_MAP_INFO_GRID_MAP_HPP_

#include <chrono>
#include <vector>
#include <string>

#include "openbot/common/port.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/map/grid_map/grid_map_msgs/grid_map_info.hpp"

namespace grid_map {
namespace grid_map_msgs {

struct GridMap
{
    // Grid map header
    GridMapInfo info;

    //  Grid map layer names.
    std::vector<std::string> layers;

    //  Grid map basic layer names (optional). The basic layers
    //  determine which layers from `layers` need to be valid
    //  in order for a cell of the grid map to be valid.
    std::vector<std::string> basic_layers;

    //  Grid map data.
    // std::vector<::openbot::common::std_msgs::Float32MultiArray> data;

    //  Row start index (default 0).
    ::openbot::uint16 outer_start_index;

    //  Column start index (default 0).
    ::openbot::uint16 inner_start_index;
};

}  // grid_map_msgs
}  // namespace grid_map

#endif  // OPENBOT_MAP_GRID_MAP_GRID_MAP_INFO_GRID_MAP_HPP_




