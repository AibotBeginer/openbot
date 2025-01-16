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
#include <unordered_map>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"

#include "openbot/common/macros.hpp"
#include "openbot/common/io/msgs.hpp"
#include "openbot/system/navigation/naviagtor/navigate_to_pose.hpp"
#include "openbot/system/navigation/proto/bt_navigator.pb.h"

namespace openbot {
namespace system { 
namespace navigation { 

class PoseNavigatorComponent final : public ::apollo::cyber::Component<> 
{
public:
    using GoalMsg = common::geometry_msgs::PoseStamped;

    PoseNavigatorComponent() = default;
    ~PoseNavigatorComponent() = default;

    bool Init() override;

private:

    bool InitConfig();
    bool InitIO();
    bool LaunchNavigator();

    void Run();

    void OnGoalReceived(const std::shared_ptr<GoalMsg>& msg);

    // navigator;
    NavigateToPoseNavigator::SharedPtr navigator_{nullptr};

    // nav config 
    std::shared_ptr<openbot::navigation::NavigationConfig> nav_config_{nullptr};

    // cyber node topics
    std::shared_ptr<apollo::cyber::Reader<GoalMsg>> start_goal_listener_ = nullptr;
    std::shared_ptr<apollo::cyber::Reader<GoalMsg>> cancel_goal_listener_ = nullptr;

    std::string nav_start_goal_topic_ = "";
    std::string nav_cancel_goal_topic_ = "";

    std::future<void> async_result_;
    std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(PoseNavigatorComponent)

}  // namespace navigation
}  // namespace system
}  // namespace openbot
