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

#include "openbot/system/navigation/behavior_tree/plugins/action/smoother_selector_node.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

SmootherSelector::SmootherSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
    node_ = config().blackboard->get<std::shared_ptr<apollo::cyber::Node>>("node");
    getInput("topic_name", topic_name_);
    smoother_selector_reader_ = node_->CreateReader<common::std_msgs::String>(
        topic_name_, 
        [this](const std::shared_ptr<common::std_msgs::String>& msg) {
            CallbackSmootherSelect(msg);
        });
}

BT::NodeStatus SmootherSelector::tick()
{
    // This behavior always use the last selected smoother received from the topic input.
    // When no input is specified it uses the default smoother.
    // If the default smoother is not specified then we work in "required smoother mode":
    // In this mode, the behavior returns failure if the smoother selection is not received from
    // the topic input.
    if (last_selected_smoother_.empty()) {
        std::string default_smoother;
        getInput("default_smoother", default_smoother);
        if (default_smoother.empty()) {
            return BT::NodeStatus::FAILURE;
        } else {
            last_selected_smoother_ = default_smoother;
        }
    }

    setOutput("selected_smoother", last_selected_smoother_);

    return BT::NodeStatus::SUCCESS;
}

void SmootherSelector::CallbackSmootherSelect(const std::shared_ptr<common::std_msgs::String> msg)
{
    last_selected_smoother_ = msg->data();
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot