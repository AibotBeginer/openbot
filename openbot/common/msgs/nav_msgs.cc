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