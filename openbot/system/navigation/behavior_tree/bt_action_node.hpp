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

#include "behaviortree_cpp/action_node.h"
#include "openbot/common/utils/logging.hpp"
#include "openbot/system/navigation/behavior_tree/bt_conversions.hpp"

namespace openbot {
namespace system { 
namespace navigation { 
namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 */
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::BtActionNode constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    BtActionNode(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_request_(true)
    {
        // node_ = config().blackboard->template get<std::shared_ptr<::apollo::cyber::Node>>("node");
    
        // Get the required items from the blackboard
        bt_loop_duration_ = config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
        server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
        getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

        // Initialize the input and output messages
        request_ = typename ActionT::Request();
        response_ = typename ActionT::Response();

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        CreateActionClient(action_name_);

        // Give the derive class a chance to do any initialization
       LOG(INFO) << xml_tag_name.c_str() << " BtActionNode initialized";
    }

    BtActionNode() = delete;

    virtual ~BtActionNode() {}

    /**
     * @brief Create instance of an action client
     * @param action_name Action name to create client for
     */
    void CreateActionClient(const std::string& action_name)
    {
    }

    /**
     * @brief Any subclass of BtActionNode that accepts parameters must provide a
     * providedPorts method and call providedBasicPorts in it.
     * @param addition Additional ports to add to BT port list
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
            BT::PortsList basic = {
                BT::InputPort<std::string>("server_name", "Action server name"),
                BT::InputPort<std::chrono::milliseconds>("server_timeout")
            };
            basic.insert(addition.begin(), addition.end());
            return basic;
    }

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    // Derived classes can override any of the following methods to hook into the
    // processing for the action: on_tick, on_wait_for_result, and on_success

    /**
     * @brief Function to perform some user-defined operation on tick
     * Could do dynamic checks, such as getting updates to values on the blackboard
     */
    virtual void OnTick()
    {
    }

    /**
     * @brief Function to perform some user-defined operation after a timeout
     * waiting for a result that hasn't been received yet. Also provides access to
     * the latest feedback message from the action server. Feedback will be nullptr
     * in subsequent calls to this function if no new feedback is received while waiting for a result.
     * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
     */
    virtual void OnWaitForResult(std::shared_ptr<const typename ActionT::Response>/*response*/)
    {
    }

    /**
     * @brief Function to perform some user-defined operation upon successful
     * completion of the action. Could put a value on the blackboard.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
     */
    virtual BT::NodeStatus OnSuccess()
    {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Function to perform some user-defined operation when the action is aborted.
     * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
     */
    virtual BT::NodeStatus OnAborted()
    {
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Function to perform some user-defined operation when the action is cancelled.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
     */
    virtual BT::NodeStatus OnCancelled()
    {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override
    {
        // first step to be done only at the beginning of the Action
        if (status() == BT::NodeStatus::IDLE) 
        {
            // setting the status to RUNNING to notify the BT Loggers (if any)
            setStatus(BT::NodeStatus::RUNNING);

            // reset the flag to send the request or not, allowing the user the option to set it in on_tick
            should_send_request_ = true;

            // user defined callback, may modify "should_send_request_".
            OnTick();

            if (!should_send_request_) {
                return BT::NodeStatus::FAILURE;
            }

            SendNewRequest();
        }
    }

    /**
     * @brief The other (optional) override required by a BT action. In this case, we
     * make sure to cancel the ROS2 action if it is still running.
     */
    void halt() override
    {
    }

protected:
    /**
     * @brief Function to check if current request should be cancelled
     * @return bool True if current request should be cancelled, false otherwise
     */
    bool ShouldCancelRequest()
    {
        return true;
    }

    /**
     * @brief Function to send new request to action server
     */
    void SendNewRequest()
    {
    }

    /**
     * @brief Function to increment recovery count on blackboard if this node wraps a recovery
     */
    void IncrementRecoveryCount()
    {
        int recovery_count = 0;
        config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
        recovery_count += 1;
        config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
    }

    std::string action_name_;
    std::shared_ptr<::apollo::cyber::Client<typename ActionT::Request, typename ActionT::Response>> action_client_;

    // All cyber actions have a request and a result
    typename ActionT::Request request_;
    typename ActionT::Response response_;
    bool request_updated_{false};
    bool request_result_available_{false};

    // The node that will be used for any cyber operations
    std::shared_ptr<::apollo::cyber::Node> node_;

    // The timeout value while waiting for response from a server when a
    // new action request is sent or canceled
    std::chrono::milliseconds server_timeout_;

    // The timeout value for BT loop execution
    std::chrono::milliseconds bt_loop_duration_;
    
    // Can be set in on_tick or on_wait_for_result to indicate if a request should be sent.
    bool should_send_request_;
};


}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot
