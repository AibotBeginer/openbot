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


#ifndef OPENBOT_PLANNING_GLOBAL_PLANNER_SERVERHPP_
#define OPENBOT_PLANNING_GLOBAL_PLANNER_SERVERHPP_

#include <memory>
#include <string>

#include "openbot/common/macros.hpp"
// #include "openbot/common/proto/nav_msgs/path.pb.h"
// #include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

// #include "class_loader/class_loader.hpp"


namespace openbot {
namespace planning { 

class PlannerServer
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(PlannerServer);

    /**
     * @brief A constructor for openbot::planning::PlannerServer
     * @param options Additional options to control creation of the node.
     */
    explicit PlannerServer();

    /**
     * @brief Destructor for openbot::planning::PlannerServer
     */
    ~PlannerServer();

private:


};

}  // namespace planning 
}  // namespace openbot

#endif  // OPENBOT_PLANNING_GLOBAL_PLANNER_SERVERHPP_
