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
#include <set>
#include <vector>

#include "cyber/cyber.h"

#include "openbot/common/io/msgs.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace BT {

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.

/**
 * @brief Parse XML string to geometry_msgs::msg::Point
 * @param key XML string
 * @return geometry_msgs::msg::Point
 */
template<>
inline openbot::common::geometry_msgs::Point convertFromString(const StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    openbot::common::geometry_msgs::Point position;
    position.set_x(BT::convertFromString<double>(parts[0]));
    position.set_y(BT::convertFromString<double>(parts[1]));
    position.set_z(BT::convertFromString<double>(parts[2]));
    return position;
  }
}

}  // namespace BT