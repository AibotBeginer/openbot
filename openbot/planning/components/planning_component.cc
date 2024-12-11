
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


#include "openbot/planning/components/planning_component.hpp"
#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace planning { 

bool PlanningComponent::Init() 
{
    LOG(INFO) << "Load PlanningComponent ....";
    return true;
}

bool PlanningComponent::Proc(const std::shared_ptr<::openbot::planning::proto::GlobalPlan>& plan) 
{
    return true;
}

}  // namespace planning 
}  // namespace openbot