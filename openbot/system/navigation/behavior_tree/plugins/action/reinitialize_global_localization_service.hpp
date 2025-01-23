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

#include "openbot/system/navigation/behavior_tree/bt_service_node.hpp"
#include "openbot/system/navigation/proto/reinitialize_global_localization.pb.h"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wrapsopenbot::navigation::ReinitializeGlobalLocalization
 */
class ReinitializeGlobalLocalizationService : public BtServiceNode<openbot::navigation::ReinitializeGlobalLocalization>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ReinitializeGlobalLocalizationService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ReinitializeGlobalLocalizationService(
        const std::string& service_node_name,
        const BT::NodeConfiguration& conf);
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot