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

#include "openbot/system/navigation/behavior_tree/plugins/action/reinitialize_global_localization_service.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

ReinitializeGlobalLocalizationService::ReinitializeGlobalLocalizationService(
  const std::string& service_node_name,
  const BT::NodeConfiguration& conf)
: BtServiceNode<openbot::navigation::ReinitializeGlobalLocalization>(service_node_name, conf)
{}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<openbot::system::navigation::behavior_tree::ReinitializeGlobalLocalizationService>(
        "ReinitializeGlobalLocalization");
}