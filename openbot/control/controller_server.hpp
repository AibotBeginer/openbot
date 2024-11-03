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


#ifndef OPENBOT_CONTROL_CONTROLLER_SERVER_HPP_
#define OPENBOT_CONTROL_CONTROLLER_SERVER_HPP_

#include <memory>
#include <string>

#include "openbot/common/macros.hpp"
// #include "openbot/common/proto/nav_msgs/path.pb.h"
// #include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

// #include "pluginlib/class_loader.hpp"
// #include "pluginlib/class_list_macros.hpp"

namespace openbot {
namespace control { 

class ControllerServer
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(ControllerServer);

    /**
     * @brief A constructor for openbot::planning::ControllerServer
     * @param options Additional options to control creation of the node.
     */
    explicit ControllerServer();

    /**
     * @brief Destructor for openbot::planning::ControllerServer
     */
    ~ControllerServer();

private:


};

}  // namespace control
}  // namespace openbot

#endif  // OPENBOT_CONTROL_CONTROLLER_SERVER_HPP_
