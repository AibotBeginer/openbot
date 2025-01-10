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

#include "cyber/cyber.h"

#include "openbot/common/io/msgs.hpp"
#include "behaviortree_cpp/action_node.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief The ProgressCheckerSelector behavior is used to switch the progress checker
 * of the controller server. It subscribes to a topic "progress_checker_selector"
 * to get the decision about what progress_checker must be used. It is usually used before of
 * the FollowPath. The selected_progress_checker output port is passed to progress_checker_id
 * input port of the FollowPath
 */
class ProgressCheckerSelector : public BT::SyncActionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ProgressCheckerSelector
     *
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf  BT node configuration
     */
    ProgressCheckerSelector(
        const std::string& xml_tag_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {
        BT::InputPort<std::string>(
            "default_progress_checker",
            "the default progress_checker to use if there is not any external topic message received."),

        BT::InputPort<std::string>(
            "topic_name",
            "progress_checker_selector",
            "the input topic name to select the progress_checker"),

        BT::OutputPort<std::string>(
            "selected_progress_checker",
            "Selected progress_checker by subscription")
        };
    }

private:
    /**
     * @brief Function to perform some user-defined operation on tick
     */
    BT::NodeStatus tick() override;

    /**
     * @brief callback function for the progress_checker_selector topic
     *
     * @param msg the message with the id of the progress_checker_selector
     */
    void CallbackProgressCheckerSelect(const std::shared_ptr<common::std_msgs::String> msg);

    std::shared_ptr<::apollo::cyber::Reader<common::std_msgs::String>> progress_checker_selector_sub_reader_;

    std::string last_selected_progress_checker_;

    std::shared_ptr<apollo::cyber::Node> node_;

    std::string topic_name_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot