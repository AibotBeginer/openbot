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

#ifndef OPENBOT_COMMON_MSGS_BUILTIN_INTERFACES_HPP_
#define OPENBOT_COMMON_MSGS_BUILTIN_INTERFACES_HPP_

// std_msgs
#include "openbot/common/proto/builtin_interfaces/time.pb.h"
#include "openbot/common/port.hpp"

namespace openbot {
namespace common {
namespace builtin_interfaces {

struct Time 
{
    // The seconds component, valid over all int32 values.
    uint32 sec;

    // The nanoseconds component, valid in the range [0, 10e9).
    uint32 nanosec;
};

// Converts 'Time' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::builtin_interfaces::Time ToProto(const Time& data);

// Converts 'proto' to openbot::common::proto::Header.
Time FromProto(const openbot::common::proto::builtin_interfaces::Time& proto);

}  // namespace builtin_interfaces
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_BUILTIN_INTERFACES_HPP_
