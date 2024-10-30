#include "openbot/common/msgs/builtin_interfaces.hpp"

namespace openbot {
namespace common {
namespace builtin_interfaces {

// Converts 'Time' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::builtin_interfaces::Time ToProto(const Time& data)
{
    openbot::common::proto::builtin_interfaces::Time proto;
    proto.set_sec(data.sec);
    proto.set_nanosec(data.nanosec);
    return proto;
}

// Converts 'proto' to openbot::common::proto::Header.
Time FromProto(const openbot::common::proto::builtin_interfaces::Time& proto)
{
    return Time {
        proto.sec(),
        proto.nanosec()   
    };
}

}  // namespace builtin_interfaces
}  // namespace common
}  // namespace openbot