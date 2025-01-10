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

#include <string>
#include <memory>

#include "openbot/common/io/msgs.hpp"
#include "behaviortree_cpp/action_node.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class TruncatePath : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::TruncatePath constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    TruncatePath(
        const std::string& xml_tag_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<common::nav_msgs::Path>("input_path", "Original Path"),
            BT::OutputPort<common::nav_msgs::Path>("output_path", "Path truncated to a certain distance"),
            BT::InputPort<double>("distance", 1.0, "distance"),
        };
    }

private:
    /**
     * @brief The other (optional) override required by a BT action.
     */
    void halt() override {}

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    double distance_;
};


}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot