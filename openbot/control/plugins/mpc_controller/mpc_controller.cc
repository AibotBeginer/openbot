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

#include "openbot/control/plugins/mpc_controller/mpc_controller.hpp"

namespace openbot {
namespace control {
namespace plugins {
namespace mpc_controller {

MpcController::~MpcController()
{

}

void MpcController::Configure()
{

}


void MpcController::Cleanup() 
{
  
}

void MpcController::Activate() 
{

}


void MpcController::Deactivate() 
{

}

void MpcController::SetPlan(const common::proto::nav_msgs::Path & path) 
{

}

common::proto::geometry_msgs::TwistStamped MpcController::ComputeVelocityCommands(
  const common::proto::geometry_msgs::PoseStamped& pose,
  const common::proto::geometry_msgs::Twist& velocity) 
{
  common::proto::geometry_msgs::TwistStamped cmd_vel;
  return cmd_vel;
}

void MpcController::SetSpeedLimit(const double& speed_limit, const bool& percentage) 
{

}

}  // namespace mpc_controller
}  // namespace plugins
}  // namespace control
}  // namespace openbot