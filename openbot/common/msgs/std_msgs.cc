#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/msgs/builtin_interfaces.hpp"

namespace openbot {
namespace common {
namespace std_msgs {

// Converts 'Header' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::std_msgs::Header ToProto(const Header& data)
{
    openbot::common::proto::std_msgs::Header proto;
    proto.set_frame_id(data.frame_id);
    *proto.mutable_stamp() = builtin_interfaces::ToProto(data.stamp);
    return proto;
}

// Converts 'proto' to openbot::common::proto::Header.
Header FromProto(const openbot::common::proto::std_msgs::Header& proto)
{
    Header data;
    data.stamp = builtin_interfaces::FromProto(proto.stamp());
    data.frame_id = proto.frame_id();
    return data;
}

}  // namespace std_msgs
}  // namespace common
}  // namespace openbot