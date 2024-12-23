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
            HandleSensorImage(image);
        });

    channel_ = ::grpc::CreateChannel("127.0.0.1:8009", ::grpc::InsecureChannelCredentials());
    grpc_client_ = std::make_shared<grpc::GrpcClientImpl>(channel_);
    LOG(INFO) << "GrpcComponent::Init finished";
    return true;
}

void GrpcComponent::HandleSensorImage(const std::shared_ptr<::openbot_bridge::common_msgs::Image>& msgs)
{
    LOG(INFO) << "recevie image data";
    grpc_client_->SendMsgToGrpc(msgs);
}

}  // namespace compontents
}  // namespace planning 
}  // namespace openbot