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


#include "openbot/control/plugins/dwa_controller.hpp"

namespace openbot {
namespace controller { 
namespace plugins { 

DWAController::~DWAController()
{

}

void DWAController::Configure()
{

}


void DWAController::Cleanup() 
{
  
}

void DWAController::Activate() 
{

}


void DWAController::Deactivate() 
{

}

void DWAController::SetPlan(const common::proto::nav_msgs::Path & path) 
{

}

common::proto::geometry_msgs::TwistStamped DWAController::ComputeVelocityCommands(
  const common::proto::geometry_msgs::PoseStamped& pose,
  const common::proto::geometry_msgs::Twist& velocity) 
{
  common::proto::geometry_msgs::TwistStamped cmd_vel;
  return cmd_vel;
}

void DWAController::SetSpeedLimit(const double& speed_limit, const bool& percentage) 
{

}

}  // namespace plugins
}  // namespace control
}  // namespace openbot