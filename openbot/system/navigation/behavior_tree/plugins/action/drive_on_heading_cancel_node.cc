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

#include "openbot/system/navigation/behavior_tree/plugins/action/drive_on_heading_cancel_node.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

DriveOnHeadingCancel::DriveOnHeadingCancel(
  const std::string& xml_tag_name,
  const std::string& action_name,
  const BT::NodeConfiguration& conf)
: BtCancelActionNode<openbot::navigation::DriveOnHeading>(xml_tag_name, action_name, conf)
{
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<openbot::system::navigation::behavior_tree::DriveOnHeadingCancel>(
            name, "drive_on_heading", config);
    };

    factory.registerBuilder<openbot::system::navigation::behavior_tree::DriveOnHeadingCancel>(
        "CancelDriveOnHeading", builder);
}