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

#include "openbot/common/msgs/builtin_interfaces.hpp"

namespace openbot {
namespace common {
namespace builtin_interfaces {

// Overload the << operator for the Time struct
std::ostream& operator<<(std::ostream& os, const Time& time) 
{
    os << "Time(sec: " << time.sec << ", nanosec: " << time.nanosec << ")";
    return os;
}

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