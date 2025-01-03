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

#include "openbot/system/navigation/naviagtor/navigate_to_pose.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system { 
namespace navigation { 

std::string NavigateToPoseNavigator::GetDefaultBTFilepath()
{
    return config_->behavior_tree().default_xml_behavior_trees();
}

bool NavigateToPoseNavigator::GoalReceived(
    const std::shared_ptr<typename ActionT::Request> request)
{
    return true;
}

void NavigateToPoseNavigator::OnLoop()
{
    LOG(INFO) << "Run Function: NavigateToPoseNavigator::OnLoop()";
}

void NavigateToPoseNavigator::OnPreempt(
    const std::shared_ptr<typename ActionT::Request> request)
{

}

void NavigateToPoseNavigator::GoalCompleted()
{

}

void NavigateToPoseNavigator::InitializeGoalPose(
    const std::shared_ptr<typename ActionT::Request> request)
{

}

}  // namespace navigation
}  // namespace system
}  // namespace openbot
