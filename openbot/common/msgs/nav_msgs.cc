#include "openbot/common/msgs/nav_msgs.hpp"


namespace openbot {
namespace common {
namespace nav_msgs {

proto::nav_msgs::Path ToProto(const Path& data)
{
    proto::nav_msgs::Path proto;
    *proto.mutable_header() = ToProto(data.header);
    for (int i = 0; i < data.poses.size(); ++i) {
        *proto.mutable_poses(i) = ToProto(data.poses[i]);
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