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