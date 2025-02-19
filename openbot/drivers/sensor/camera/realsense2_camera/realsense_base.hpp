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

#if defined (ACCELERATE_GPU_WITH_GLSL)
#include <gl_window.h>
#endif

#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// #include <librealsense2/rs.hpp>
// #include <librealsense2/rsutil.h>
// #include <librealsense2/hpp/rs_processing.hpp>
// #include <librealsense2/rs_advanced_mode.hpp>

#include "openbot/drivers/sensor/camera/realsense2_camera/constants.hpp"
#include "openbot/drivers/sensor/camera/realsense2_camera/realsense_camera_msgs.hpp"


namespace realsense2_camera {




}  // namespace realsense2_camera
