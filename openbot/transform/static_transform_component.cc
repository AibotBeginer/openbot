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


#include "openbot/transform/static_transform_component.hpp"

#include "yaml-cpp/yaml.h"

#include "openbot/common/adapters/adapter_gflags.hpp"
#include "openbot/common/utils/message_util.hpp"

namespace openbot {
namespace transform {

bool StaticTransformComponent::Init() 
{
  if (!GetProtoConfig(&conf_)) {
    AERROR << "Parse conf file failed, " << ConfigFilePath();
    return false;
  }
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(FLAGS_tf_static_topic);
  attr.mutable_qos_profile()->CopyFrom(
      apollo::cyber::transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  writer_ = node_->CreateWriter<common::geometry_msgs::TransformStampeds>(attr);
  SendTransforms();
  return true;
}

void StaticTransformComponent::SendTransforms() {
  std::vector<common::geometry_msgs::TransformStamped> tranform_stamped_vec;
  for (auto& extrinsic_file : conf_.extrinsic_file()) {
    if (extrinsic_file.enable()) {
      AINFO << "Broadcast static transform, frame id ["
            << extrinsic_file.frame_id() << "], child frame id ["
            << extrinsic_file.child_frame_id() << "]";
      common::geometry_msgs::TransformStamped transform;
      if (ParseFromYaml(extrinsic_file.file_path(), &transform)) {
        tranform_stamped_vec.emplace_back(transform);
      }
    }
  }
  SendTransform(tranform_stamped_vec);
}

bool StaticTransformComponent::ParseFromYaml(
    const std::string& file_path, common::geometry_msgs::TransformStamped* transform_stamped) {
  if (!apollo::cyber::common::PathExists(file_path)) {
    AERROR << "Extrinsic yaml file does not exist: " << file_path;
    return false;
  }
  YAML::Node tf = YAML::LoadFile(file_path);
  try {
    transform_stamped->mutable_header()->set_frame_id(tf["header"]["frame_id"].as<std::string>());
    transform_stamped->set_child_frame_id(tf["child_frame_id"].as<std::string>());
    // translation
    auto translation = transform_stamped->mutable_transform()->mutable_translation();
    translation->set_x(tf["transform"]["translation"]["x"].as<double>());
    translation->set_y(tf["transform"]["translation"]["y"].as<double>());
    translation->set_z(tf["transform"]["translation"]["z"].as<double>());
    // rotation
    auto rotation = transform_stamped->mutable_transform()->mutable_rotation();
    rotation->set_x(tf["transform"]["rotation"]["x"].as<double>());
    rotation->set_y(tf["transform"]["rotation"]["y"].as<double>());
    rotation->set_z(tf["transform"]["rotation"]["z"].as<double>());
    rotation->set_w(tf["transform"]["rotation"]["w"].as<double>());
  } catch (...) {
    AERROR << "Extrinsic yaml file parse failed: " << file_path;
    return false;
  }
  return true;
}

void StaticTransformComponent::SendTransform(
    const std::vector<common::geometry_msgs::TransformStamped>& msgtf) {
  for (auto it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in) {
    bool match_found = false;
    for (auto& it_msg : *transform_stampeds_.mutable_transforms()) {
      if (it_in->child_frame_id() == it_msg.child_frame_id()) {
        it_msg = *it_in;
        match_found = true;
        break;
      }
    }
    if (!match_found) {
      *transform_stampeds_.add_transforms() = *it_in;
    }
  }

  // TODO(duyongquan)
  // common::util::FillHeader(node_->Name(), &transform_stampeds_);
  writer_->Write(transform_stampeds_);
}

}  // namespace transform
}  // namespace apollo
