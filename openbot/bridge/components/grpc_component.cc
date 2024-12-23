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

#include "openbot/bridge/components/grpc_component.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace bridge { 
namespace compontents { 

bool GrpcComponent::Init()
{
    sensor_image_reader_ = node_->CreateReader<::openbot_bridge::common_msgs::Image>(
        "/openbot/sensor/camera/test/image", 
        [this](const std::shared_ptr<openbot_bridge::common_msgs::Image>& image) {
            HandleSensorImageMessages(image);
        });

    pointcloud_reader_ = node_->CreateReader<::openbot_bridge::common_msgs::PointCloud>(
        "/openbot/map/matterport/pointcloud", 
        [this](const std::shared_ptr<openbot_bridge::common_msgs::PointCloud>& msgs) {
            HandlePointCloudMessages(msgs);
        });

    grpc_config_ = std::make_shared<::openbot::bridge::grpc::GRPCConfig>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, grpc_config_.get())) {
      return false;
    }
    LOG(INFO) << "GRPC config: " << grpc_config_->DebugString();
    std::string ip_addr = grpc_config_->host() + ":" + grpc_config_->port();
    channel_ = ::grpc::CreateChannel(ip_addr, ::grpc::InsecureChannelCredentials());
    grpc_client_ = std::make_shared<grpc::GrpcClientImpl>(channel_);

    async_result_ = apollo::cyber::Async(&GrpcComponent::Run, this);
    LOG(INFO) << "GrpcComponent::Init finished";
    return true;
}

void GrpcComponent::Run()
{
    running_.exchange(true);
    // while (!apollo::cyber::IsShutdown()) {
    //     apollo::cyber::SleepFor(std::chrono::seconds(1));
    // }
}

void GrpcComponent::HandleSensorImageMessages(const std::shared_ptr<::openbot_bridge::common_msgs::Image>& msgs)
{
    LOG(INFO) << "recevie image data";
    grpc_client_->SendMsgToGrpc(msgs);
}

void GrpcComponent::HandlePointCloudMessages(const std::shared_ptr<::openbot_bridge::common_msgs::PointCloud>& msgs)
{
    LOG(INFO) << "recevie pointcloud data";
    grpc_client_->SendMsgToGrpc(msgs);
}

GrpcComponent::~GrpcComponent()
{
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

}  // namespace compontents
}  // namespace planning 
}  // namespace openbot