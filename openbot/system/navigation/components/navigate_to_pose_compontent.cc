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

#include "openbot/system/navigation/components/navigate_to_pose_compontent.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system { 
namespace navigation { 

bool PoseNavigatorComponent::Init() 
{
    if (!InitConfig()) {
        AERROR << "Init Config failed.";
        return false;
    }

    if (!InitIO()) {
        AERROR << "Init IO failed.";
        return false;
    }

    if (!LaunchNavigator()) {
        AERROR << "Launch navigator failed.";
        return false;
    }

    async_result_ = apollo::cyber::Async(&PoseNavigatorComponent::Run, this);
    return true;
}

bool PoseNavigatorComponent::InitConfig()
{
    nav_config_ = std::make_shared<openbot::navigation::NavigationConfig>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, nav_config_.get())) {
        return false;
    }
    return true;
}

bool PoseNavigatorComponent::InitIO()
{
    // // start_goal_listener
    // start_goal_listener_ = this->node_->CreateReader<GoalMsg>(
    //     nav_start_goal_topic_, 
    //     [this](const std::shared_ptr<GoalMsg>& msgs) {
    //         OnGoalReceived(msgs);
    // });

    // cancel_goal_listener
    // cancel_goal_listener_ = 
    return true;
}

bool PoseNavigatorComponent::LaunchNavigator()
{
    navigator_ = std::make_shared<NavigateToPoseNavigator>(node_, nav_config_.get());
    return true;
}

void PoseNavigatorComponent::OnGoalReceived(const std::shared_ptr<GoalMsg>& msg)
{
    if (msg == nullptr) {
        return;
    }

    if (navigator_ == nullptr) {
        return;
    }

    // AINFO << "Goal pose: " << msg->
    navigator_->SetTargetGoalPose(msg);
}

void PoseNavigatorComponent::Run()
{
    apollo::cyber::SleepFor(std::chrono::seconds(1));
    running_.exchange(true);
    while (!apollo::cyber::IsShutdown()) {
        navigator_->ExecuteCallback();
        LOG(INFO) << "ExecuteCallback";
        apollo::cyber::SleepFor(std::chrono::seconds(1));
    }
}

}  // namespace navigation
}  // namespace system
}  // namespace openbot
