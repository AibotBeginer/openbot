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

#include <vector>
#include <array>
#include <string>

#include "openbot_bridge/common_msgs/sensor_msgs.pb.h"

namespace realsense2_camera {

//  Cross-stream extrinsics: encodes the topology describing how the different devices are oriented
struct Extrinsics
{
    std::array<double, 9> rotation;     // Column - major 3x3 rotation matrix
    std::array<double, 3> translation;  // Three-element translation vector, in meters
};

struct IMUInfo 
{
    std::array<double, 12> data;
    std::array<double, 3> noise_variances;
    std::array<double, 3> bias_variances;
};

struct Metadata
{
    std::string json_data;
};

struct RGBD
{
    ::openbot_bridge::common_msgs::CameraInfo rgb_camera_info;
    ::openbot_bridge::common_msgs::CameraInfo depth_camera_info;
    ::openbot_bridge::common_msgs::Image rgb;
    ::openbot_bridge::common_msgs::Image depth;
};

}  // namespace realsense2_camera 
