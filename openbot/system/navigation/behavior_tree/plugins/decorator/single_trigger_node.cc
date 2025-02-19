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

#include "openbot/system/navigation/behavior_tree/plugins/decorator/single_trigger_node.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

SingleTrigger::SingleTrigger(const std::string & name, const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
     first_time_(true)
{
}

BT::NodeStatus SingleTrigger::tick()
{
    if (status() == BT::NodeStatus::IDLE) {
        first_time_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    if (first_time_) {
        const BT::NodeStatus child_state = child_node_->executeTick();

        switch (child_state) {
        case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
            first_time_ = false;
            return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
            first_time_ = false;
            return BT::NodeStatus::FAILURE;

        default:
            first_time_ = false;
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<openbot::system::navigation::behavior_tree::SingleTrigger>("SingleTrigger");
}