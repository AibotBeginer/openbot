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

#include "openbot/common/msgs/nav_msgs.hpp"
#include "openbot/common/msgs/std_msgs.hpp"

namespace openbot {
namespace common {
namespace nav_msgs {

proto::nav_msgs::GridCells ToProto(const GridCells& data)
{
    proto::nav_msgs::GridCells proto;
    return proto;
}

GridCells FromProto(const proto::nav_msgs::GridCells& proto)
{
    GridCells data;
    return data;
}

// MapMetaData
proto::nav_msgs::MapMetaData ToProto(const MapMetaData& data)
{
    proto::nav_msgs::MapMetaData proto;
    return proto;
}

MapMetaData FromProto(const proto::nav_msgs::MapMetaData& proto)
{
    MapMetaData data;
    return data;
}

// OccupancyGrid
proto::nav_msgs::OccupancyGrid ToProto(const OccupancyGrid& data)
{
    proto::nav_msgs::OccupancyGrid proto;
    return proto;
}

OccupancyGrid FromProto(const proto::nav_msgs::OccupancyGrid& proto)
{
    OccupancyGrid data;
    return data;
}

proto::nav_msgs::Odometry ToProto(const Odometry& data)
{
    proto::nav_msgs::Odometry proto;
    return proto;
}

Odometry FromProto(const proto::nav_msgs::Odometry& proto)
{
    Odometry data;
    return data;
}

proto::nav_msgs::Path ToProto(const Path& data)
{
    proto::nav_msgs::Path proto;
    *proto.mutable_header() = ToProto(data.header);
    for (int i = 0; i < data.poses.size(); ++i) {
        *proto.add_poses() = ToProto(data.poses[i]);
    }
    return proto;
}

Path FromProto(const proto::nav_msgs::Path& proto)
{
    Path data;
    data.header = std_msgs::FromProto(proto.header());
    for (int i = 0; i < proto.poses_size(); ++i) {
        data.poses.push_back(geometry_msgs::FromProto(proto.poses(i)));
    }
    return data;
}

}  // namespace nav_msgs
}  // namespace common
}  // namespace openbot