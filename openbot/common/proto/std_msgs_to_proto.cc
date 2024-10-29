#include "openbot/common/proto/std_msgs_to_proto.hpp"

namespace openbot {
namespace common {
namespace proto {
namespace std_msgs {

// Converts 'Header' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::std_msgs::Header ToProto(const Header& data)
{
    openbot::common::proto::std_msgs::Header proto;
    return proto;
}

// Converts 'proto' to openbot::common::proto::Header.
Header FromProto(const openbot::common::proto::std_msgs::Header& proto)
{
    return Header {
        // proto.stamp(),
        // proto.frame_id
    };
}

}  // namespace std_msgs
}  // namespace proto
}  // namespace common
}  // namespace openbot