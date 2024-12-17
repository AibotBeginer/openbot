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

#include "openbot/transform/transform_broadcaster.hpp"
#include "openbot/common/adapters/adapter_gflags.hpp"

namespace openbot {
namespace transform {

TransformBroadcaster::TransformBroadcaster(
    const std::shared_ptr<apollo::cyber::Node>& node)
    : node_(node) 
{
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_tf_topic);
  writer_ = node_->CreateWriter<common::geometry_msgs::TransformStampeds>(attr);
}

void TransformBroadcaster::SendTransform(const common::geometry_msgs::TransformStamped& transform) {
  std::vector<common::geometry_msgs::TransformStamped> transforms;
  transforms.emplace_back(transform);
  SendTransform(transforms);
}

void TransformBroadcaster::SendTransform(
    const std::vector<common::geometry_msgs::TransformStamped>& transforms) {
  auto message = std::make_shared<common::geometry_msgs::TransformStampeds>();
  *message->mutable_transforms() = {transforms.begin(), transforms.end()};
  writer_->Write(message);
}

}  // namespace transform
}  // namespace openbot
