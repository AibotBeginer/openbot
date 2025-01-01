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

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

namespace openbot {
namespace system { 
namespace navigation {
namespace behavior_tree {

/**
 * @enum nav2_behavior_tree::BtStatus
 * @brief An enum class representing BT execution status
 */
enum class BtStatus 
{ 
    SUCCEEDED, 
    FAILED, 
    CANCELED 
};

/**
 * @class nav2_behavior_tree::BehaviorTreeEngine
 * @brief A class to create and handle behavior trees
 */
class BehaviorTreeEngine
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::BehaviorTreeEngine
     * @param plugin_libraries vector of BT plugin library names to load
     */
    explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
    virtual ~BehaviorTreeEngine() {}

    /**
     * @brief Function to execute a BT at a specific rate
     * @param tree BT to execute
     * @param onLoop Function to execute on each iteration of BT execution
     * @param cancelRequested Function to check if cancel was requested during BT execution
     * @param loopTimeout Time period for each iteration of BT execution
     * @return nav2_behavior_tree::BtStatus Status of BT execution
     */
    BtStatus Run(
        BT::Tree * tree,
        std::function<void()> OnLoop,
        std::function<bool()> CancelRequested,
        std::chrono::milliseconds loop_timeout = std::chrono::milliseconds(10));

    /**
     * @brief Function to create a BT from a XML string
     * @param xml_string XML string representing BT
     * @param blackboard Blackboard for BT
     * @return BT::Tree Created behavior tree
     */
    BT::Tree CreateTreeFromText(
        const std::string & xml_string,
        BT::Blackboard::Ptr blackboard);

    /**
     * @brief Function to create a BT from an XML file
     * @param file_path Path to BT XML file
     * @param blackboard Blackboard for BT
     * @return BT::Tree Created behavior tree
     */
    BT::Tree CreateTreeFromFile(
        const std::string & file_path,
        BT::Blackboard::Ptr blackboard);

    /**
     * @brief Function to explicitly reset all BT nodes to initial state
     * @param root_node Pointer to BT root node
     */
    void HaltAllActions(BT::TreeNode * root_node);

protected:
    // The factory that will be used to dynamically construct the behavior tree
    BT::BehaviorTreeFactory factory_;
};

}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot