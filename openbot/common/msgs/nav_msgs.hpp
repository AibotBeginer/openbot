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

#ifndef OPENBOT_COMMON_MSGS_NAV_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_NAV_MSGS_HPP_

#include <vector>
#include <string>

#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/msgs/geometry_msgs.hpp"
#include "openbot/common/msgs/builtin_interfaces.hpp"

#include "openbot/common/proto/nav_msgs/grid_cells.pb.h" 
#include "openbot/common/proto/nav_msgs/map_meta_data.pb.h"
#include "openbot/common/proto/nav_msgs/occupancy_grid.pb.h" 
#include "openbot/common/proto/nav_msgs/odometry.pb.h"
#include "openbot/common/proto/nav_msgs/path.pb.h" 


namespace openbot {
namespace common {
namespace nav_msgs {

// An array of cells in a 2D grid
struct GridCells
{
    std_msgs::Header header;

    // Width of each cell
    float cell_width;

    // Height of each cell
    float cell_height;

    // Each cell is represented by the Point at the center of the cell
    std::vector<geometry_msgs::Point> cells;
};

// This hold basic information about the characteristics of the OccupancyGrid
struct MapMetaData
{
    // The time at which the map was loaded
    builtin_interfaces::Time map_load_time;
    
    // The map resolution [m/cell]
    float resolution;
    
    // Map width [cells]
    uint32 width;
    
    // Map height [cells]
    uint32 height;
    
    // The origin of the map [m, m, rad].  This is the real-world pose of the
    // bottom left corner of cell (0,0) in the map.
    geometry_msgs::Pose origin;
};

struct OccupancyGrid
{
    // This represents a 2-D grid map
    std_msgs::Header header;

    // MetaData for the map
    MapMetaData info;

    // The map data, in row-major order, starting with (0,0). 
    // Cell (1, 0) will be listed second, representing the next cell in the x direction. 
    // Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
    // The values inside are application dependent, but frequently, 
    // 0 represents unoccupied, 1 represents definitely occupied, and
    // -1 represents unknown. 
    std::vector<int32> data;
};

struct Odometry
{
    // This represents an estimate of a position and velocity in free space.
    // The pose in this message should be specified in the coordinate frame given by header.frame_id
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id

    // Includes the frame id of the pose parent.
    std_msgs::Header header;

    // Frame id the pose points to. The twist is in this coordinate frame.
    std::string child_frame_id;

    // Estimated pose that is typically relative to a fixed world frame.
    geometry_msgs::PoseWithCovariance pose;

    // Estimated linear and angular velocity relative to child_frame_id.
    geometry_msgs::TwistWithCovariance twist;
};

// An array of poses that represents a Path for a robot to follow.
struct Path
{
    // Indicates the frame_id of the path.
    std_msgs::Header header;

    // Array of poses to follow.
    std::vector<geometry_msgs::PoseStamped> poses;
};

// GridCells
proto::nav_msgs::GridCells ToProto(const GridCells& data);
GridCells FromProto(const proto::nav_msgs::GridCells& proto);

// MapMetaData
proto::nav_msgs::MapMetaData ToProto(const MapMetaData& data);
MapMetaData FromProto(const proto::nav_msgs::MapMetaData& proto);

// OccupancyGrid
proto::nav_msgs::OccupancyGrid ToProto(const OccupancyGrid& data);
OccupancyGrid FromProto(const proto::nav_msgs::OccupancyGrid& proto);

// Odometry
proto::nav_msgs::Odometry ToProto(const Odometry& data);
Odometry FromProto(const proto::nav_msgs::Odometry& proto);

// Path
proto::nav_msgs::Path ToProto(const Path& data);
Path FromProto(const proto::nav_msgs::Path& proto);


}  // namespace nav_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_NAV_MSGS_HPP_
