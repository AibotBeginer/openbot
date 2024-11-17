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


#include "openbot/control/controller_server.hpp"
#include "openbot/common/utils/logging.hpp"

// #include "openbot/common/proto/nav_msgs/path.pb.h"
// #include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

namespace openbot {
namespace control { 

ControllerServer::ControllerServer()
{
}

ControllerServer::~ControllerServer()
{
    goal_checkers_.clear();
    controllers_.clear();
}

common::Status ControllerServer::OnConfigure()
{
    return common::Status::OK();
}

common::Status ControllerServer::OnActivate()
{
    return common::Status::OK();
}

common::Status ControllerServer::OnDeactivate()
{
    return common::Status::OK();
}

common::Status ControllerServer::OnCleanup()
{
    return common::Status::OK();
}

bool ControllerServer::FindControllerId(const std::string& c_name, std::string& current_goal_checker)
{
    if (goal_checkers_.find(c_name) == goal_checkers_.end()) 
    {
        if (goal_checkers_.size() == 1 && c_name.empty()) {
            // LOG_ONCE(WARNING) << "No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded " 
            //                   << goal_checker_ids_concat_.c_str() 
            //                   << "This warning will appear once.";
            current_goal_checker = goal_checkers_.begin()->first;
        } else {
            LOG(ERROR) << "FollowPath called with goal_checker name " 
                       << c_name.c_str() << " in parameter 'current_goal_checker', which does not exist. Available goal checkers are: " 
                       << goal_checker_ids_concat_.c_str();
            return false;
        }
    } else {
        LOG(INFO) << "Selected goal checker: " << c_name.c_str();
        current_goal_checker = c_name;
    }

    return true;
}

void ControllerServer::ComputeControl()
{
    std::lock_guard<std::mutex> lock(dynamic_params_lock_);
    LOG(INFO) << "Received a goal, begin computing control effort.";
}

void ControllerServer::SetPlannerPath(const common::nav_msgs::Path& path)
{
    LOG(INFO) << "Providing path to the controller " <<  current_controller_.c_str();
    if (path.poses.empty()) {
        LOG(ERROR) << "Invalid path, Path is empty.";
        return;
    }
    controllers_[current_controller_]->SetPlan(path);

    end_pose_ = path.poses.back();
    end_pose_.header.frame_id = path.header.frame_id;
    goal_checkers_[current_goal_checker_].reset();

    LOG(INFO) << "Path end point is ("
         << end_pose_.pose.position.x << ", " 
         << end_pose_.pose.position.y << ")";

    current_path_ = path;
}

void ControllerServer::UpdateGlobalPath()
{

}

void ControllerServer::PublishVelocity(const common::geometry_msgs::TwistStamped& velocity)
{
}

void ControllerServer::PublishZeroVelocity()
{
    common::geometry_msgs::TwistStamped velocity;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 0;
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
}

bool ControllerServer::IsGoalReached()
{
    common::geometry_msgs::PoseStamped pose;

    if (!GetRobotPose(pose)) {
        return false;
    }

    return true;
}

bool ControllerServer::GetRobotPose(common::geometry_msgs::PoseStamped& pose)
{
    common::geometry_msgs::PoseStamped current_pose;
    pose = current_pose;
    return true;
}

// void ControllerServer::SpeedLimitCallback(const common::nav_msgs::SpeedLimit::SharedPtr msg)
// {
//   ControllerMap::iterator it;
//   for (it = controllers_.begin(); it != controllers_.end(); ++it) {
//     it->second->setSpeedLimit(msg->speed_limit, msg->percentage);
//   }
// }


}  // namespace controller 
}  // namespace openbot