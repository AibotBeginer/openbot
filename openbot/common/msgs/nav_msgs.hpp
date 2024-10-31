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

#ifndef OPENBOT_COMMON_MSGS_NAV_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_NAV_MSGS_HPP_

#include <vector>

#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/msgs/geometry_msgs.hpp"

#include "openbot/common/proto/nav_msgs/grid_cells.pb.h" 
#include "openbot/common/proto/nav_msgs/map_meta_data.pb.h"
#include "openbot/common/proto/nav_msgs/occupancy_grid.pb.h" 
#include "openbot/common/proto/nav_msgs/odometry.pb.h"
#include "openbot/common/proto/nav_msgs/path.pb.h" 


namespace openbot {
namespace common {
namespace nav_msgs {

struct Odometry
{

};

struct Path
{
    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};

// Path
proto::nav_msgs::Path ToProto(const Path& data);
Path FromProto(const proto::nav_msgs::Path& proto);


}  // namespace nav_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_NAV_MSGS_HPP_
