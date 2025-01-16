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

#include "openbot/system/navigation/behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include "behaviortree_cpp/utils/shared_library.h"
#include "openbot/common/utils/logging.hpp"
#include "openbot/system/navigation/common/library_config.hpp"


namespace openbot {
namespace system { 
namespace navigation {
namespace behavior_tree {

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string>& plugin_libraries)
{
    BT::SharedLibrary loader;
    for (const auto& p : plugin_libraries) {
        factory_.registerFromPlugin(BehaviorTreeLibraryDirectory() + loader.getOSName(p));
    }
}

BtStatus BehaviorTreeEngine::Run(
  BT::Tree* tree,
  std::function<void()> OnLoop,
  std::function<bool()> CancelRequested,
  std::chrono::milliseconds loop_timeout)
{
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    // Loop until something happens with cyberRT or the node completes
    try {
        while (::apollo::cyber::OK() && result == BT::NodeStatus::RUNNING) 
        {
            if (CancelRequested()) {
                tree->haltTree();
                return BtStatus::CANCELED;
            }

            result = tree->tickOnce();

            OnLoop();

            ::apollo::cyber::SleepFor(loop_timeout);
        }
    } catch (const std::exception & ex) {
        LOG(ERROR) << "Behavior tree threw exception: " <<  ex.what() 
                   << " Exiting with failure.";
        return BtStatus::FAILED;
    }

    return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::CreateTreeFromText(const std::string& xml_string, BT::Blackboard::Ptr blackboard)
{
    AINFO << "-------------------------------------1-----";
    
    if (blackboard == nullptr) {
        AINFO << "-------------------------------------2-----";
    }


    return factory_.createTreeFromText(xml_string);
}

BT::Tree BehaviorTreeEngine::CreateTreeFromFile(const std::string& file_path, BT::Blackboard::Ptr blackboard)
{
    return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void BehaviorTreeEngine::HaltAllActions(BT::Tree& tree)
{
    // this halt signal should propagate through the entire tree.
    tree.haltTree();
}

}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot