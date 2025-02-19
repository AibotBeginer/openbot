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

#include "openbot/system/navigation/naviagtor/navigate_through_poses.hpp"

namespace openbot {
namespace system { 
namespace navigation { 

NavigateThroughPosesNavigator::NavigateThroughPosesNavigator(
    const std::shared_ptr<apollo::cyber::Node>& node, 
    const openbot::navigation::NavigationConfig* config)
    : node_{node},
      config_{config}
{

}

std::string NavigateThroughPosesNavigator::GetDefaultBTFilepath()
{
    std::string default_bt_xml_filename;
    // auto node = parent_node.lock();

    // if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
    //     std::string pkg_share_dir =
    //     ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    //     node->declare_parameter<std::string>(
    //     "default_nav_to_pose_bt_xml",
    //     pkg_share_dir +
    //     "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");
    // }

    // node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);


    return default_bt_xml_filename;
}

}  // namespace navigation
}  // namespace system
}  // namespace openbot
