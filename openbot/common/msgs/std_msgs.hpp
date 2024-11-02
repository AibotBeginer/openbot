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

#ifndef OPENBOT_COMMON_MSGS_STD_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_STD_MSGS_HPP_

#include <string>

#include "openbot/common/proto/std_msgs/header.pb.h"
#include "openbot/common/msgs/builtin_interfaces.hpp"

namespace openbot {
namespace common {
namespace std_msgs {

struct Header
{
    builtin_interfaces::Time stamp;

    // Transform frame with which this data is associated.
    std::string frame_id;
};

// Converts 'Header' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::std_msgs::Header ToProto(const Header& data);

// Converts 'proto' to openbot::common::proto::Header.
Header FromProto(const openbot::common::proto::std_msgs::Header& proto);

}  // namespace std_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_STD_MSGS_HPP_