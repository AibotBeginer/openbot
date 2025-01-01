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

#include <memory>
#include <string>
#include <vector>

#include "openbot/system/navigation/common/navigator.hpp"
#include "openbot/system/navigation/naviagtor/navigate_to_pose.hpp"
#include "openbot/system/navigation/naviagtor/navigate_through_poses.hpp"


namespace openbot {
namespace system { 
namespace navigation { 

/**
 * @class BtNavigator
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtNavigator
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(BtNavigator);

    /**
     * @brief A constructor for BtNavigator class
     * @param options Additional options to control creation of the node.
     */
    explicit BtNavigator();
    /**
     * @brief A destructor for BtNavigator class
     */
    ~BtNavigator();

protected:

    // To handle all the BT related execution
    std::unique_ptr<Navigator<openbot::navigation::NavigateToPose>> pose_navigator_;
    std::unique_ptr<Navigator<openbot::navigation::NavigateThroughPoses>> poses_navigator_;

    NavigatorMuxer plugin_muxer_;
};

}  // namespace navigation
}  // namespace system
}  // namespace openbot
