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

#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <deque>

#include "openbot/common/macros.hpp"
#include "openbot/common/io/msgs.hpp"
#include "openbot/common/utils/logging.hpp"

#include "cyber/cyber.h"

namespace openbot {
namespace system { 
namespace navigation { 

/**
 * @class OdomSmoother
 * Wrapper for getting smooth odometry readings using a simple moving avergae.
 * Subscribes to the topic with a mutex.
 */
class OdomSmoother
{
public:
    /**
     * @brief Constructor that subscribes to an Odometry topic
     * @param parent NodeHandle for creating subscriber
     * @param filter_duration Duration for odom history (seconds)
     * @param odom_topic Topic on which odometry should be received
     */
    explicit OdomSmoother(
        const std::weak_ptr<apollo::cyber::Node>& parent,
        double filter_duration = 0.3,
        const std::string & odom_topic = "odom");

    /**
     * @brief Get twist msg from smoother
     * @return twist Twist msg
     */
    inline common::geometry_msgs::Twist GetTwist() {return vel_smooth_.twist();}

    /**
     * @brief Get twist stamped msg from smoother
     * @return twist TwistStamped msg
     */
    inline common::geometry_msgs::TwistStamped GetTwistStamped() {return vel_smooth_;}

protected:
    /**
     * @brief Callback of odometry subscriber to process
     * @param msg Odometry msg to smooth
     */
    void OdomCallback(std::shared_ptr<common::nav_msgs::Odometry> msg);

    /**
     * @brief Update internal state of the smoother after getting new data
     */
    void UpdateState();

    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_reader_;
    common::nav_msgs::Odometry odom_cumulate_;
    common::geometry_msgs::TwistStamped vel_smooth_;
    std::mutex odom_mutex_;

    // rclcpp::Duration odom_history_duration_;
    std::deque<common::nav_msgs::Odometry> odom_history_;
};

}  // namespace navigation
}  // namespace system
}  // namespace openbot
