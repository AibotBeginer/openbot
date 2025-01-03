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

#include "openbot/common/io/msgs.hpp"
#include "openbot/common/macros.hpp"
#include "openbot/system/navigation/naviagtor/navigator.hpp"
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
  : public Navigator<openbot::navigation::NavigateThroughPoses>
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
      const openbot::navigation::NavigationConfig* config) 
      : Navigator(node), config_{config} {}

    /**
     * @brief A subscription and callback to handle the topic-based goal published
     * from rviz
     * @param pose Pose received via atopic
     */
    void OnGoalPoseReceived(const std::shared_ptr<common::geometry_msgs::PoseStamped>& pose);

    /**
     * @brief Get action name for this navigator
     * @return string Name of action server
     */
    std::string GetName() override { return std::string("navigate_through_poses"); }

    /**
     * @brief Get navigator's default BT
     * @param node WeakPtr to the lifecycle node
     * @return string Filepath to default XML
     */
    std::string GetDefaultBTFilepath() override;

protected:
    /**
     * @brief A callback to be called when a new goal is received by the BT action server
     * Can be used to check if goal is valid and put values on
     * the blackboard which depend on the received goal
     * @param goal Action template's goal message
     * @return bool if goal was received successfully to be processed
     */
    bool GoalReceived(const std::shared_ptr<typename ActionT::Request> request) override;

    /**
     * @brief A callback that defines execution that happens on one iteration through the BT
     * Can be used to publish action feedback
     */
    void OnLoop() override;

    /**
     * @brief A callback that is called when a preempt is requested
     */
    void OnPreempt(const std::shared_ptr<typename ActionT::Request> request) override;

    /**
     * @brief A callback that is called when a the action is completed, can fill in
     * action result message or indicate that this action is done.
     * @param result Action template result message to populate
     * @param final_bt_status Resulting status of the behavior tree execution that may be
     * referenced while populating the result.
     */
    void GoalCompleted() override;

    /**
     * @brief Goal pose initialization on the blackboard
     * @param goal Action template's goal message to process
     */
    void InitializeGoalPose(const std::shared_ptr<typename ActionT::Request> request);

    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;
private:
    // config
    const openbot::navigation::NavigationConfig* config_;
};

}  // namespace navigation
}  // namespace system
}  // namespace openbot
