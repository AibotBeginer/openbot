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
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "openbot/common/io/msgs.hpp"
#include "openbot/common/utils/logging.hpp"
#include "openbot/common/service_wrapper/server_wrapper.hpp"
#include "openbot/system/navigation/behavior_tree/behavior_tree_engine.hpp"

namespace openbot {
namespace system { 
namespace navigation {
namespace behavior_tree {

template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
    std::shared_ptr<::apollo::cyber::Node>& node,
    const std::string& action_name,
    const std::vector<std::string>& plugin_lib_names,
    const std::string& default_bt_xml_filename,
    OnGoalReceivedCallback on_goal_received_callback,
    OnLoopCallback on_loop_callback,
    OnPreemptCallback on_preempt_callback,
    OnCompletionCallback on_completion_callback)
    : client_node_(node),
      action_name_(action_name),
      default_bt_xml_filename_(default_bt_xml_filename),
      plugin_lib_names_(plugin_lib_names),
      on_goal_received_callback_(on_goal_received_callback),
      on_loop_callback_(on_loop_callback),
      on_preempt_callback_(on_preempt_callback),
      on_completion_callback_(on_completion_callback)
{
    // Create Action Server
    action_server_ = std::make_shared<ActionServer>(
        node, action_name_, std::bind(&BtActionServer<ActionT>::ExecuteCallback, this));


    // Create the class that registers our custom nodes and executes the BT
    bt_ = std::make_unique<BehaviorTreeEngine>(plugin_lib_names_);

    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();

    // Put items on the blackboard
    blackboard_->set<std::shared_ptr<::apollo::cyber::Node>>("node", client_node_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT
    blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);

    // Load behavior tree 
    if (!LoadBehaviorTree(default_bt_xml_filename_)) {
        LOG(ERROR) << "Error loading XML file: " << default_bt_xml_filename_;
    }
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{
    action_server_.reset();
    plugin_lib_names_.clear();
    current_bt_xml_filename_.clear();
    blackboard_.reset();
    bt_->HaltAllActions(tree_);
    bt_.reset();
}

template<class ActionT>
bool BtActionServer<ActionT>::LoadBehaviorTree(const std::string& bt_xml_filename)
{
    // Empty filename is default for backward compatibility
    auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    // Use previous BT if it is the existing one
    if (current_bt_xml_filename_ == filename) {
        LOG(INFO) << "BT will not be reloaded as the given xml is already loaded";
        return true;
    }

    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(filename);

    if (!xml_file.good()) {
        LOG(ERROR) << "Couldn't open input XML file: " << filename;
        return false;
    }

    auto xml_string = std::string(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input
    try {
        tree_ = bt_->CreateTreeFromText(xml_string, blackboard_);
        for (auto& subtree : tree_.subtrees) {
            auto& blackboard = subtree->blackboard;
            blackboard->set("node", client_node_);
            blackboard->set<std::shared_ptr<::apollo::cyber::Node>>("node", client_node_);
            blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
            blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        }
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception when loading BT: " << e.what();
        return false;
    }

    current_bt_xml_filename_ = filename;
    return true;
}

template<class ActionT>
void BtActionServer<ActionT>::ExecuteCallback()
{
    // if (!on_goal_received_callback_(action_server_->GetCurrentRequest())) {
    //     action_server_->TerminateCurrent();
    //     return;
    // }

    LOG(INFO) << "ExecuteCallback() .....》》》》》》》》》》》》》》 1";

    auto is_canceling = [&]() {
        if (action_server_ == nullptr) {
            LOG(WARNING) << "Action server unavailable. Canceling.";
            return true;
        }
        if (!action_server_->IsServerActive()) {
            LOG(INFO) << "Action server is inactive. Canceling.";
            return true;
        }
        return action_server_->IsCancelRequested();
    };

    LOG(INFO) << "ExecuteCallback() .....》》》》》》》》》》》》》》 2";
    auto on_loop = [&]() {
        // if (action_server_->IsPreemptRequested() && on_preempt_callback_) {
        //     on_preempt_callback_(action_server_->GetPendingRequest());
        // }
        on_loop_callback_();
    };


    LOG(INFO) << "ExecuteCallback() .....》》》》》》》》》》》》》》 3";

    // // Execute the BT that was previously created in the configure step
    // BtStatus rc = bt_->Run(&tree_, on_loop, is_canceling, bt_loop_duration_);

    // // Make sure that the Bt is not in a running state from a previous execution
    // // note: if all the ControlNodes are implemented correctly, this is not needed.
    // bt_->HaltAllActions(tree_);

    // // Give server an opportunity to populate the result message or simple give
    // // an indication that the action is complete.
    // auto result = std::make_shared<typename ActionT::Response>();
    // on_completion_callback_(result, rc);

    // switch (rc) 
    // {
    //     case BtStatus::SUCCEEDED:
    //         LOG(INFO) << "Goal succeeded";
    //         // action_server_->succeeded_current(result);
    //         break;

    //     case BtStatus::FAILED:
    //         LOG(INFO) << "Goal failed";
    //         // action_server_->terminate_current(result);
    //         break;

    //     case BtStatus::CANCELED:
    //         LOG(INFO) << "Goal canceled";
    //         // action_server_->terminate_all(result);
    //         break;
    // }
}

}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot