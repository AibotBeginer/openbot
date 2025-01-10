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
#include <chrono>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/common/log.h"

#include "behaviortree_cpp/action_node.h"
#include "openbot/common/service_wrapper/client_wrapper.hpp"
#include "openbot/system/navigation/behavior_tree/bt_conversions.hpp"

namespace openbot {
namespace system { 
namespace navigation {
namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action for cancelling BT node
 * @tparam ActionT Type of action
 */
template<class ActionT>
class BtCancelActionNode : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::BtCancelActionNode constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    BtCancelActionNode(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
    {
        node_ = config().blackboard->template get<std::shared_ptr<apollo::cyber::Node>>("node");

        // Get the required items from the blackboard
        BT::getInputOrBlackboard("server_timeout", server_timeout_);
        wait_for_service_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        CreateActionClient(action_name_);

        // Give the derive class a chance to do any initialization
        AINFO << xml_tag_name.c_str() << " BtCancelActionNode initialized";
    }

    BtCancelActionNode() = delete;

    virtual ~BtCancelActionNode()
    {
    }

    /**
     * @brief Create instance of an action client
     * @param action_name Action name to create client for
     */
    void CreateActionClient(const std::string& action_name)
    {
        // // Now that we have the ROS node to use, create the action client for this BT action
        // action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

        // // Make sure the server is actually there before continuing
        // // RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());

        // if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) {
        // AERROR(
        //     node_->get_logger(), "\"%s\" action server not available after waiting for %.2fs",
        //     action_name.c_str(), wait_for_service_timeout_.count() / 1000.0);
        // throw std::runtime_error(
        //         std::string("Action server ") + action_name +
        //         std::string(" not available"));
        // }
    }

    /**
     * @brief Any subclass of BtCancelActionNode that accepts parameters must provide a
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

    void halt() override
    {
    }

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override
    {
        // setting the status to RUNNING to notify the BT Loggers (if any)
        setStatus(BT::NodeStatus::RUNNING);

        // Cancel all the goals specified before 10ms from current time
        // to avoid async communication error

        // rclcpp::Time goal_expiry_time = node_->now() - std::chrono::milliseconds(10);

        // auto future_cancel = action_client_->async_cancel_goals_before(goal_expiry_time);

        // if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        // rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     AERROR << "Failed to cancel the action server for " << action_name_.c_str();
        //     return BT::NodeStatus::FAILURE;
        // }
        return BT::NodeStatus::SUCCESS;
    }

protected:
    std::string action_name_;
    std::shared_ptr<::apollo::cyber::Client<typename ActionT::Request, typename ActionT::Response>> action_client_;

    // The node that will be used for any cyber operations
    std::shared_ptr<apollo::cyber::Node> node_;

    // The timeout value while waiting for response from a server when a
    // new action goal is canceled
    std::chrono::milliseconds server_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;
};

}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot
