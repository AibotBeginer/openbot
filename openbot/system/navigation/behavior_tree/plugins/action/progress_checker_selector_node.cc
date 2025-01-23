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

#include "openbot/system/navigation/behavior_tree/plugins/action/progress_checker_selector_node.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

ProgressCheckerSelector::ProgressCheckerSelector(
  const std::string& name,
  const BT::NodeConfiguration& conf)
: BT::SyncActionNode(name, conf)
{
    node_ = config().blackboard->get<std::shared_ptr<apollo::cyber::Node>>("node");
    getInput("topic_name", topic_name_);
    progress_checker_selector_sub_reader_ = node_->CreateReader<common::std_msgs::String>(
        topic_name_, 
        [this](const std::shared_ptr<common::std_msgs::String>& msg) {
            CallbackProgressCheckerSelect(msg);
        });
}

BT::NodeStatus ProgressCheckerSelector::tick()
{
    // This behavior always use the last selected progress checker received from the topic input.
    // When no input is specified it uses the default goaprogressl checker.
    // If the default progress checker is not specified then we work in
    // "required progress checker mode": In this mode, the behavior returns failure if the progress
    // checker selection is not received from the topic input.
    if (last_selected_progress_checker_.empty()) {
        std::string default_progress_checker;
        getInput("default_progress_checker", default_progress_checker);
        if (default_progress_checker.empty()) {
            return BT::NodeStatus::FAILURE;
        } else {
            last_selected_progress_checker_ = default_progress_checker;
        }
    }

    setOutput("selected_progress_checker", last_selected_progress_checker_);

    return BT::NodeStatus::SUCCESS;
}

void ProgressCheckerSelector::CallbackProgressCheckerSelect(const std::shared_ptr<common::std_msgs::String> msg)
{
  last_selected_progress_checker_ = msg->data();
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<openbot::system::navigation::behavior_tree::ProgressCheckerSelector>(
        "ProgressCheckerSelector");
}