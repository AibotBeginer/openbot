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

#include "openbot/drivers/components/camera_component.hpp"
#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace drivers {
namespace components {

bool CameraComponent::Init() 
{
  // raw_writer_ = node_->CreateWriter<Image>(camera_config_->raw_channel_name());
  async_result_ = apollo::cyber::Async(&CameraComponent::run, this);
  return true;
}

void CameraComponent::run() 
{
  running_.exchange(true);
  while (!apollo::cyber::IsShutdown()) {
   
    // raw_writer_->Write(raw_image_for_compress);
    LOG(INFO) << "Publish images";
    apollo::cyber::SleepFor(std::chrono::seconds(1));
  }
}

CameraComponent::~CameraComponent() 
{
  if (running_.load()) {
    running_.exchange(false);
    // free(raw_image_->image);
    async_result_.wait();
  }
}

}  // namespace components
}  // namespace drivers
}  // namespace openbot
