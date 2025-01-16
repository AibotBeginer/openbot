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

#include <gtest/gtest.h>

#include <vector>
#include <limits>
#include <fstream>

#include "cyber/cyber.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "openbot/common/utils/logging.hpp"
#include "openbot/system/navigation/common/library_config.hpp"
#include "openbot/system/navigation/behavior_tree/plugins/dummy_nodes.hpp"

namespace openbot {
namespace system { 
namespace navigation {
namespace behavior_tree {

static const char* xml_text = R"(
 <root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="battery_ok"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

TEST(BehaviorTreeEngineTest, LoadTree) 
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // Load dynamically a plugin and register the TreeNodes it contains
    // it automated the registering step. 
    factory.registerFromPlugin(BehaviorTreeLibraryDirectory() + "libnav_dummy_nodes_bt_node.so");
                                                              
    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();
}

TEST(BehaviorTreeEngineTest, LoadTree2) 
{
    std::vector<std::string> plugin_libs = {
        "nav_dummy_nodes_bt_node"
    };

    auto bt = BehaviorTreeEngine(plugin_libs);

    auto blackboard = BT::Blackboard::create();

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    // auto tree = bt.CreateTreeFromText(xml_text, blackboard);

    auto tree = bt.CreateTreeFromFile("/opt/openbot/share/openbot/system/navigation/conf/behavior_trees/dummy_test.xml", blackboard);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();
}

}  // namespace behavior_tree 
}  // namespace navigation
}  // namespace system
}  // namespace openbot