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

#ifndef OPENBOT_COMMON_INTERPOLATION_SPHERICAL_LINEAR_INTERPOLATION_HPP_
#define OPENBOT_COMMON_INTERPOLATION_SPHERICAL_LINEAR_INTERPOLATION_HPP_

#include "openbot/common/interpolation/interpolation_utils.hpp"
#include "openbot/common/msgs/msgs.hpp"

#include <vector>

namespace openbot {
namespace common {
namespace interpolation {

geometry_msgs::Quaternion slerp(
  const geometry_msgs::Quaternion & src_quat, const geometry_msgs::Quaternion & dst_quat,
  const double ratio);

std::vector<geometry_msgs::Quaternion> slerp(
  const std::vector<double> & base_keys,
  const std::vector<geometry_msgs::Quaternion> & base_values,
  const std::vector<double> & query_keys);

}  // namespace interpolation
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_INTERPOLATION_SPHERICAL_LINEAR_INTERPOLATION_HPP_
