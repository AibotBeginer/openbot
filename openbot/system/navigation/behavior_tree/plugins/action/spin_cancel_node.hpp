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

#include "openbot/common/io/msgs.hpp"
#include "openbot/system/navigation/behavior_tree/bt_cancel_action_node.hpp"
#include "openbot/system/navigation/proto/spin.pb.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait
 */
class SpinCancel : public BtCancelActionNode<openbot::navigation::Spin>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::WaitAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    SpinCancel(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
        });
    }
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot