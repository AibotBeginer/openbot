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

#define REALSENSE_ROS_MAJOR_VERSION    4
#define REALSENSE_ROS_MINOR_VERSION    55
#define REALSENSE_ROS_PATCH_VERSION    1

namespace realsense2_camera {

const uint16_t RS400_PID        = 0x0ad1; // PSR
const uint16_t RS410_PID        = 0x0ad2; // ASR
const uint16_t RS415_PID        = 0x0ad3; // ASRC
const uint16_t RS430_PID        = 0x0ad4; // AWG
const uint16_t RS430_MM_PID     = 0x0ad5; // AWGT
const uint16_t RS_USB2_PID      = 0x0ad6; // USB2
const uint16_t RS420_PID        = 0x0af6; // PWG
const uint16_t RS420_MM_PID     = 0x0afe; // PWGT
const uint16_t RS410_MM_PID     = 0x0aff; // ASR
const uint16_t RS400_MM_PID     = 0x0b00; // PSR
const uint16_t RS430_MM_RGB_PID = 0x0b01; // AWGCT
const uint16_t RS460_PID        = 0x0b03; // DS5U
const uint16_t RS435_RGB_PID    = 0x0b07; // AWGC
const uint16_t RS435i_RGB_PID   = 0x0B3A; // AWGC_MM
const uint16_t RS416_RGB_PID    = 0x0B52; // F416 RGB
const uint16_t RS430i_PID       = 0x0b4b; // D430i
const uint16_t RS405_PID        = 0x0B5B; // DS5U
const uint16_t RS455_PID        = 0x0B5C; // D455
const uint16_t RS457_PID        = 0xABCD; // D457   

const bool ALLOW_NO_TEXTURE_POINTS = false;
const bool ORDERED_PC     = false;
const bool SYNC_FRAMES    = false;
const bool ENABLE_RGBD    = false;

const bool PUBLISH_TF     = true;
const double TF_PUBLISH_RATE = 0; // Static transform
const double DIAGNOSTICS_PERIOD = 0.0;

const std::string IMAGE_QOS    = "SYSTEM_DEFAULT";
const std::string DEFAULT_QOS  = "DEFAULT";
const std::string HID_QOS      = "SENSOR_DATA";

const bool HOLD_BACK_IMU_FOR_FRAMES = false;

const std::string DEFAULT_BASE_FRAME_ID            = "link";
const std::string DEFAULT_IMU_OPTICAL_FRAME_ID     = "camera_imu_optical_frame";

const float ROS_DEPTH_SCALE = 0.001;

}  // namespace realsense2_camera
