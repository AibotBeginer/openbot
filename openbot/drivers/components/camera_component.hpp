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

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include <opencv2/opencv.hpp>

#include "openbot_bridge/sensor_msgs/sensor_image.pb.h"
#include "openbot/drivers/proto/camera_config.pb.h"

namespace openbot {
namespace drivers {
namespace components {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class CameraComponent : public Component<> 
{
 public:
  bool Init() override;
  ~CameraComponent();

private:
  void run();

  void ToImage(const cv::Mat& cvImage, openbot_bridge::sensor_msgs::Image& imageMessage);

  std::shared_ptr<Writer<openbot_bridge::sensor_msgs::Image>> writer_ = nullptr;
  std::shared_ptr<Writer<openbot_bridge::sensor_msgs::Image>> raw_writer_ = nullptr;
  std::shared_ptr<openbot::drivers::camera::config::Config> camera_config_;
  uint32_t spin_rate_ = 200;
  uint32_t device_wait_ = 2000;
  int index_ = 0;
  int buffer_size_ = 16;
  const int32_t MAX_IMAGE_SIZE = 20 * 1024 * 1024;
  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(CameraComponent)

}  // namespace components
}  // namespace drivers
}  // namespace openbot
