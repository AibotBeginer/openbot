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

#include "openbot/system/navigation/behavior_tree/plugins/decorator/speed_controller.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

SpeedController::SpeedController(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf),
    first_tick_(false),
    period_(1.0),
    min_rate_(0.1),
    max_rate_(1.0),
    min_speed_(0.0),
    max_speed_(0.5)
{
    // node_ = config().blackboard->get<::apollo::cyber::Node>("node");

    getInput("min_rate", min_rate_);
    getInput("max_rate", max_rate_);
    getInput("min_speed", min_speed_);
    getInput("max_speed", max_speed_);

    if (min_rate_ <= 0.0 || max_rate_ <= 0.0) 
    {
        std::string err_msg = "SpeedController node cannot have rate <= 0.0";
        LOG(ERROR) << err_msg.c_str();
        throw BT::BehaviorTreeException(err_msg);
    }

    d_rate_ = max_rate_ - min_rate_;
    d_speed_ = max_speed_ - min_speed_;

    std::string odom_topic;
    // node_->get_parameter_or("odom_topic", odom_topic, std::string("odom"));
    // odom_smoother_ = config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>(
    // "odom_smoother");
}

inline BT::NodeStatus SpeedController::tick()
{
    if (status() == BT::NodeStatus::IDLE) 
    {
        // Reset since we're starting a new iteration of
        // the speed controller (moving from IDLE to RUNNING)
        config().blackboard->get<std::vector<common::geometry_msgs::PoseStamped>>("goals", goals_);
        config().blackboard->get<common::geometry_msgs::PoseStamped>("goal", goal_);
        period_ = 1.0 / max_rate_;
        // start_ = node_->now();
        first_tick_ = true;
    }

    // std::vector<common::geometry_msgs::PoseStamped> current_goals;
    // config().blackboard->get<std::vector<common::geometry_msgs::PoseStamped>>("goals", current_goals);
    // common::geometry_msgs::PoseStamped current_goal;
    // config().blackboard->get<common::geometry_msgs::PoseStamped>("goal", current_goal);

    // // if (goal_ != current_goal || goals_ != current_goals) 
    // // {
    // //     // Reset state and set period to max since we have a new goal
    // //     period_ = 1.0 / max_rate_;
    // //     // start_ = node_->now();
    // //     first_tick_ = true;
    // //     goal_ = current_goal;
    // //     goals_ = current_goals;
    // // }

    setStatus(BT::NodeStatus::RUNNING);

    // // auto elapsed = node_->now() - start_;

    // // The child gets ticked the first time through and any time the period has
    // // expired. In addition, once the child begins to run, it is ticked each time
    // // 'til completion
    // if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) || elapsed.seconds() >= period_)
    // {
    //     first_tick_ = false;

    //     // update period if the last period is exceeded
    //     if (elapsed.seconds() >= period_) {
    //         updatePeriod();
    //         // start_ = node_->now();
    //     }

    //     const BT::NodeStatus child_state = child_node_->executeTick();

    //     switch (child_state) 
    //     {
    //         case BT::NodeStatus::RUNNING:
    //             return BT::NodeStatus::RUNNING;

    //         case BT::NodeStatus::SUCCESS:
    //             return BT::NodeStatus::SUCCESS;

    //         case BT::NodeStatus::FAILURE:
    //         default:
    //             return BT::NodeStatus::FAILURE;
    //     }
    // }

    return status();
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::SpeedController>("SpeedController");
}
