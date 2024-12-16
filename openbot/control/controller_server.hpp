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

#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>

#include "openbot/common/macros.hpp"
#include "openbot/control/common/controller_base.hpp"
#include "openbot/control/common/goal_checker_base.hpp"

namespace openbot {
namespace control { 

class ControllerServer
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(ControllerServer);

    using ControllerMap = std::unordered_map<std::string, Controller::SharedPtr>;
    using GoalCheckerMap = std::unordered_map<std::string, GoalChecker::SharedPtr>;

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
