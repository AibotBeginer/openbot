/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file ros_bridge_gflags.h
 * @brief The gflags used by ros bridge binary
 */

#include "openbot/tools/ros_bridge/common/ros_bridge_gflags.hpp"

namespace apollo {
namespace cyber {

// config path
DEFINE_string(bridge_conf_path,
  "ros_bridge/conf/ros_bridge_conf.pb.txt", "ros config file path");

}  // namespace cyber
}  // namespace apollo
