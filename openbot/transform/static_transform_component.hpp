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

#include <memory>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "openbot/transform/proto/static_transform_conf.pb.h"
#include "openbot/common/proto/geometry_msgs.pb.h"

namespace openbot {
namespace transform {

class StaticTransformComponent final : public apollo::cyber::Component<> 
{
 public:
  StaticTransformComponent() = default;
  ~StaticTransformComponent() = default;

 public:
  bool Init() override;

 private:
  void SendTransforms();
  void SendTransform(const std::vector<common::geometry_msgs::TransformStamped>& msgtf);
  bool ParseFromYaml(const std::string& file_path, common::geometry_msgs::TransformStamped* transform);

  openbot::static_transform::Conf conf_;
  std::shared_ptr<apollo::cyber::Writer<common::geometry_msgs::TransformStampeds>> writer_;
  common::geometry_msgs::TransformStampeds transform_stampeds_;
};

CYBER_REGISTER_COMPONENT(StaticTransformComponent)

}  // namespace transform
}  // namespace openbot
