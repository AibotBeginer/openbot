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

#include "cyber/cyber.h"

#include "openbot/common/io/msgs.hpp"
#include "openbot/common/macros.hpp"
#include "openbot/system/navigation/proto/navigate_through_poses.pb.h"
#include "openbot/system/navigation/proto/bt_navigator.pb.h"

namespace openbot {
namespace system { 
namespace navigation { 

/**
 * @class NavigateThroughPosesNavigator
 * @brief A navigator for navigating to a a bunch of intermediary poses
 */
class NavigateThroughPosesNavigator
{
public:
    using ActionT = openbot::navigation::NavigateThroughPoses;

     /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(NavigateThroughPosesNavigator);

    /**
     * @brief A constructor for NavigateToPoseNavigator
     */
    NavigateThroughPosesNavigator(
      const std::shared_ptr<apollo::cyber::Node>& node,
      const openbot::navigation::NavigationConfig* config);

private:

    /**
     * @brief Get action name for this navigator
     * @return string Name of action server
     */
    std::string GetName() { return std::string("navigate_through_poses"); }

    /**
     * @brief Get navigator's default BT
     * @param node WeakPtr to the lifecycle node
     * @return string Filepath to default XML
     */
    std::string GetDefaultBTFilepath();

    std::shared_ptr<apollo::cyber::Node> node_;

    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;

    // config
    const openbot::navigation::NavigationConfig* config_;
};

}  // namespace navigation
}  // namespace system
}  // namespace openbot
