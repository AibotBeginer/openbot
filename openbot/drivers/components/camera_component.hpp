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

  // std::shared_ptr<Writer<Image>> writer_ = nullptr;
  // std::shared_ptr<Writer<Image>> raw_writer_ = nullptr;

  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(CameraComponent)

}  // namespace components
}  // namespace drivers
}  // namespace openbot
