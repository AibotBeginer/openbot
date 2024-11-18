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


#ifndef OPENBOT_CONTROL_CONTROLLER_SERVER_HPP_
#define OPENBOT_CONTROL_CONTROLLER_SERVER_HPP_

#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>

#include "openbot/common/macros.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/status/status.hpp"
#include "openbot/control/common/controller_base.hpp"
#include "openbot/control/common/goal_checker_base.hpp"
// #include "openbot/common/proto/nav_msgs/path.pb.h"
// #include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

// #include "pluginlib/class_loader.hpp"
// #include "pluginlib/class_list_macros.hpp"

namespace openbot {
namespace control { 

class ControllerServer
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(ControllerServer);

    using ControllerMap = std::unordered_map<std::string, Controller::SharedPtr>;
    using GoalCheckerMap = std::unordered_map<std::string, GoalChecker::SharedPtr>;

    /**
     * @brief A constructor for openbot::planning::ControllerServer
     * @param options Additional options to control creation of the node.
     */
    explicit ControllerServer();

    /**
     * @brief Destructor for openbot::planning::ControllerServer
     */
    ~ControllerServer();

    /**
     *  @brief Configures controller parameters and member variables
     */
    common::Status OnConfigure();

    /**
     *  @brief Activates controller, costmap, velocity publisher and follow path 
     */
    common::Status OnActivate();

    /**
     *  @brief  Deactivates member variables
     */
    common::Status OnDeactivate();

    /**
     *  @brief  Controller and costmap clean up state is called, and resets rest of the variables
     */
    common::Status OnCleanup();

private:

    /**
     * @brief FollowPath server callback. Handles server updates and
     * spins server until goal is reached
     *
     * Provides global path to controller received from client. Twist
     * velocities for the robot are calculated and published using controller at
     * the specified rate till the goal is reached.
     */
    void ComputeControl();

    /**
     * @brief Find the valid controller ID name for the given request
     *
     * @param c_name The requested controller name
     * @param name Reference to the name to use for control if any valid available
     * @return bool Whether it found a valid controller to use
     */
    bool FindControllerId(const std::string& c_name,  std::string& current_goal_checker);

    /**
     * @brief Assigns path to controller
     * @param path Path received from action server
     */
    void SetPlannerPath(const common::nav_msgs::Path& path);

    /**
     * @brief Calculates velocity and publishes to "cmd_vel" topic
     */
    void ComputeAndPublishVelocity();

    /**
     * @brief Calls setPlannerPath method with an updated path received from
     * action server
     */
    void UpdateGlobalPath();

    /**
     * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
     * @param velocity Twist velocity to be published
     */
    void PublishVelocity(const common::geometry_msgs::TwistStamped& velocity);

    /**
     * @brief Calls velocity publisher to publish zero velocity
     */
    void PublishZeroVelocity();

    /**
     * @brief Checks if goal is reached
     * @return true or false
     */
    bool IsGoalReached();

    /**
     * @brief Obtain current pose of the robot
     * @param pose To store current pose of the robot
     * @return true if able to obtain current pose of the robot, else false
     */
    bool GetRobotPose(common::geometry_msgs::PoseStamped & pose);

    /**
     * @brief get the thresholded velocity
     * @param velocity The current velocity from odometry
     * @param threshold The minimum velocity to return non-zero
     * @return double velocity value
     */
    double GetThresholdedVelocity(double velocity, double threshold)
    {
        return (std::abs(velocity) > threshold) ? velocity : 0.0;
    }

    // /**
    //  * @brief get the thresholded Twist
    //  * @param Twist The current Twist from odometry
    //  * @return Twist Twist after thresholds applied
    //  */
    // nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
    // {
    //     nav_2d_msgs::msg::Twist2D twist_thresh;
    //     twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    //     twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    //     twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    //     return twist_thresh;
    // }

    // /**
    //  * @brief Callback for speed limiting messages
    //  * @param msg Shared pointer to nav_msgs::msg::SpeedLimit
    //  */
    // void SpeedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);

    std::mutex dynamic_params_lock_;

    // Goal Checker Plugin
    // pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
    GoalCheckerMap goal_checkers_;
    std::vector<std::string> default_goal_checker_ids_;
    std::vector<std::string> default_goal_checker_types_;
    std::vector<std::string> goal_checker_ids_;
    std::vector<std::string> goal_checker_types_;
    std::string goal_checker_ids_concat_, current_goal_checker_;
    
    // Controller Plugins
    // pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
    ControllerMap controllers_;
    std::vector<std::string> default_ids_;
    std::vector<std::string> default_types_;
    std::vector<std::string> controller_ids_;
    std::vector<std::string> controller_types_;
    std::string controller_ids_concat_, current_controller_;

    double controller_frequency_;
    double min_x_velocity_threshold_;
    double min_y_velocity_threshold_;
    double min_theta_velocity_threshold_;

    double failure_tolerance_;

    // Whether we've published the single controller warning yet
    common::geometry_msgs::PoseStamped end_pose_;

    // Current path container
    common::nav_msgs::Path current_path_;
};

}  // namespace control
}  // namespace openbot

#endif  // OPENBOT_CONTROL_CONTROLLER_SERVER_HPP_
