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

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "openbot/common/macros.hpp"
#include "openbot/common/io/msgs.hpp"
#include "openbot/common/utils/logging.hpp"
#include "openbot/planning/planner_server.hpp"
#include "openbot/system/navigation/common/odometry_utils.hpp"
#include "openbot/system/navigation/behavior_tree/bt_action_server.hpp"

#include "cyber/cyber.h"

namespace openbot {
namespace system { 
namespace navigation { 

/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */
class NavigatorMuxer
{
public:
    /**
     * @brief A Navigator Muxer constructor
     */
    NavigatorMuxer() : current_navigator_(std::string("")) {}

    /**
     * @brief Get the navigator muxer state
     * @return bool If a navigator is in progress
     */
    bool IsNavigating()
    {
        std::scoped_lock l(mutex_);
        return !current_navigator_.empty();
    }

    /**
     * @brief Start navigating with a given navigator
     * @param string Name of the navigator to start
     */
    void StartNavigating(const std::string & navigator_name)
    {
        std::scoped_lock l(mutex_);
        if (!current_navigator_.empty()) {
            LOG(ERROR) << 
                "Major error! Navigation requested while another navigation"
                " task is in progress! This likely occurred from an incorrect"
                "implementation of a navigator plugin.";
        }
        current_navigator_ = navigator_name;
    }

    /**
     * @brief Stop navigating with a given navigator
     * @param string Name of the navigator ending task
     */
    void StopNavigating(const std::string & navigator_name)
    {
        std::scoped_lock l(mutex_);
        if (current_navigator_ != navigator_name) {
            LOG(ERROR) <<
                "Major error! Navigation stopped while another navigation"
                " task is in progress! This likely occurred from an incorrect"
                "implementation of a navigator plugin.";
        } else {
            current_navigator_ = std::string("");
        }
    }

protected:
    std::string current_navigator_;
    std::mutex mutex_;
};

/**
 * @class Navigator
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 */
template<class ActionT>
class Navigator
{
public:
    // using Ptr = std::shared_ptr<openbot::system::navigation::Navigator<ActionT>>;

    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(Navigator);

    /**
     * @brief A Navigator constructor
     */
    Navigator(const std::shared_ptr<apollo::cyber::Node>& node)
        : node_{node}
    {
        plugin_muxer_ = nullptr;
    }

    /**
     * @brief Destructor for openbot::system::navigation::Navigator
     */
    virtual ~Navigator() = default;

    /**
     * @brief Get the action name of this navigator to expose
     * @return string Name of action to expose
     */
    virtual std::string GetName() = 0;

    virtual std::string GetDefaultBTFilepath() = 0;

    void Configure(
        const std::vector<std::string>& plugin_lib_names, 
        NavigatorMuxer* plugin_muxer,
        std::shared_ptr<OdomSmoother> odom_smoother)
    {
        // get the default behavior tree for this navigator
        std::string default_bt_xml_filename = GetDefaultBTFilepath();

        // Create the Behavior Tree Action Server for this navigator
        bt_action_server_ = std::make_unique<behavior_tree::BtActionServer<ActionT>>(
            node_,
            GetName(),
            plugin_lib_names,
            default_bt_xml_filename,
            std::bind(&Navigator::OnGoalReceived, this, std::placeholders::_1),
            std::bind(&Navigator::OnLoop, this),
            std::bind(&Navigator::OnPreempt, this, std::placeholders::_1),
            std::bind(&Navigator::OnCompletion, this, std::placeholders::_1, std::placeholders::_2));
    }

   
    // using OnPreemptCallback = std::function<void (const typename std::shared_ptr<typename ActionT::Request>&)>;
    // using OnCompletionCallback = std::function<bool (const typename std::shared_ptr<typename ActionT::Response>&, behavior_tree::BtStatus)>;

    //   explicit BtActionServer(
    //     std::shared_ptr<::apollo::cyber::Node>& node,
    //     const std::string& action_name,
    //     const std::vector<std::string>& plugin_lib_names,
    //     const std::string& default_bt_xml_filename,
    //     OnGoalReceivedCallback on_goal_received_callback,
    //     OnLoopCallback on_loop_callback,
    //     OnPreemptCallback on_preempt_callback,
    //     OnCompletionCallback on_completion_callback);

protected:
    /**
     * @brief An intermediate goal reception function to mux navigators.
     */
    bool OnGoalReceived(const std::shared_ptr<typename ActionT::Request>& goal)
    {
        if (plugin_muxer_->IsNavigating()) {
            LOG(ERROR) <<  "Requested navigation from " << GetName().c_str() 
                       << " while another navigator is processing rejecting request.";
            return false;
        }

        bool goal_accepted = GoalReceived(goal);

        if (goal_accepted) {
            plugin_muxer_->StartNavigating(GetName());
        }

        return goal_accepted;
    }

    /**
     * @brief An intermediate completion function to mux navigators
     */
    void OnCompletion(const std::shared_ptr<typename ActionT::Response> result, 
        const behavior_tree::BtStatus final_bt_status)
    {
        plugin_muxer_->StopNavigating(GetName());
        // GoalCompleted(result, final_bt_status);
    }

    /**
     * @brief A callback to be called when a new goal is received by the BT action server
     * Can be used to check if goal is valid and put values on
     * the blackboard which depend on the received goal
     */
    virtual bool GoalReceived(const std::shared_ptr<typename ActionT::Request> request) = 0;

    /**
     * @brief A callback that defines execution that happens on one iteration through the BT
     * Can be used to publish action feedback
     */
    virtual void OnLoop() = 0;

    /**
     * @brief A callback that is called when a preempt is requested
     */
    virtual void OnPreempt(const std::shared_ptr<typename ActionT::Request> request) = 0;

    /**
     * @brief A callback that is called when a the action is completed; Can fill in
     * action result message or indicate that this action is done.
     */
    virtual void GoalCompleted() = 0;

    std::shared_ptr<apollo::cyber::Node> node_;

private:
    std::unique_ptr<behavior_tree::BtActionServer<ActionT>> bt_action_server_;
    NavigatorMuxer* plugin_muxer_;
};


}  // namespace navigation
}  // namespace system
}  // namespace openbot
