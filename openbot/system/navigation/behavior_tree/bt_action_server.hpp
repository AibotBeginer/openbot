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

#include "cyber/cyber.h"

#include "openbot/common/io/msgs.hpp"
#include "openbot/common/service_wrapper/server_wrapper.hpp"
#include "openbot/system/navigation/behavior_tree/behavior_tree_engine.hpp"

namespace openbot {
namespace system { 
namespace navigation {
namespace behavior_tree {

template <class ActionT>
class BtActionServer
{
public:
    using ActionServer = common::ServiceWrapper<ActionT>;
    using OnGoalReceivedCallback = std::function<bool (const std::shared_ptr<typename ActionT::Request>)>;
    using OnLoopCallback = std::function<void ()>;
    using OnPreemptCallback = std::function<void (const typename std::shared_ptr<typename ActionT::Request>)>;
    using OnCompletionCallback = std::function<void (const typename std::shared_ptr<typename ActionT::Response>, const behavior_tree::BtStatus)>;

    /**
     * @brief A constructor for nav2_behavior_tree::BtActionServer class
     */
    explicit BtActionServer(
        std::shared_ptr<::apollo::cyber::Node>& node,
        const std::string& action_name,
        const std::vector<std::string>& plugin_lib_names,
        const std::string& default_bt_xml_filename,
        OnGoalReceivedCallback on_goal_received_callback,
        OnLoopCallback on_loop_callback,
        OnPreemptCallback on_preempt_callback,
        OnCompletionCallback on_completion_callback);

    /**
     * @brief A destructor for nav2_behavior_tree::BtActionServer class
     */
    ~BtActionServer();

    /**
     * @brief Replace current BT with another one
     * @param bt_xml_filename The file containing the new BT, uses default filename if empty
     * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
     * if something went wrong, and previous BT is maintained
     */
    bool LoadBehaviorTree(const std::string & bt_xml_filename = "");

    /**
     * @brief Getter function for BT Blackboard
     * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
     */
    BT::Blackboard::Ptr GetBlackboard() const
    {
        return blackboard_;
    }

    /**
     * @brief Getter function for current BT XML filename
     * @return string Containing current BT XML filename
     */
    std::string GetCurrentBTFilename() const
    {
        return current_bt_xml_filename_;
    }

    /**
     * @brief Getter function for default BT XML filename
     * @return string Containing default BT XML filename
     */
    std::string GetDefaultBTFilename() const
    {
        return default_bt_xml_filename_;
    }

    /**
     * @brief Getter function for the current BT tree
     * @return BT::Tree Current behavior tree
     */
    const BT::Tree& GetTree() const
    {
        return tree_;
    }

    /**
     * @brief Function to halt the current tree. It will interrupt the execution of RUNNING nodes
     * by calling their halt() implementation (only for Async nodes that may return RUNNING)
     */
    void HaltTree()
    {
        tree_.rootNode()->halt();
    }
    
protected:
    /**
     * @brief Action server callback
     */
    void ExecuteCallback();

    // Action name
    std::string action_name_;

    // Our action server implements the template action
    std::shared_ptr<ActionServer> action_server_;

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;

    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;

    // The XML file that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    // The wrapper class for the BT functionality
    std::unique_ptr<BehaviorTreeEngine> bt_;

    // Libraries to pull plugins (BT Nodes) from
    std::vector<std::string> plugin_lib_names_;

    // A regular, non-spinning ROS node that we can use for calls to the action client
    std::shared_ptr<apollo::cyber::Node> client_node_;

    // Duration for each iteration of BT execution
    std::chrono::milliseconds bt_loop_duration_;

    // Default timeout value while waiting for response from a server
    std::chrono::milliseconds default_server_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;

    // User-provided callbacks
    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;
};

}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot

#include "openbot/system/navigation/behavior_tree/bt_action_server_impl.hpp"  // NOLINT(build/include_order)