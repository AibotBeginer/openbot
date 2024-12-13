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


#include "openbot/smoother/components/savitzky_golay_smoother.hpp"

namespace openbot {
namespace smoother { 
namespace components { 

SavitzkyGolaySmoother::~SavitzkyGolaySmoother()
{

}

void SavitzkyGolaySmoother::Configure()
{

}

void SavitzkyGolaySmoother::Cleanup() 
{
  
}

void SavitzkyGolaySmoother::Activate() 
{

}

void SavitzkyGolaySmoother::Deactivate() 
{

}

bool SavitzkyGolaySmoother::Smooth(common::proto::nav_msgs::Path& path, const common::Duration& max_time)
{
    return true;
}

}  // namespace components
}  // namespace smoother 
}  // namespace openbot