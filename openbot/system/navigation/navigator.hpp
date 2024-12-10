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

#pragma once

#include "openbot/common/macros.hpp"
#include "openbot/planning/planner_server.hpp"

#include "cyber/cyber.h"

namespace openbot {
namespace system { 
namespace navigation { 

class Navigator
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(Navigator);

    /**
     * @brief A constructor for openbot::system::navigation::Navigator
     */
    explicit Navigator();

    /**
     * @brief Destructor for openbot::system::navigation::Navigator
     */
    ~Navigator();

private:
    // Cyber Node
    std::unique_ptr<apollo::cyber::Node> node_{nullptr};
};


}  // namespace navigation
}  // namespace system
}  // namespace openbot
