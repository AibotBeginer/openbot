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

#include <grpcpp/grpcpp.h>
#include <grpc/impl/codegen/connectivity_state.h>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"

#include "openbot/common/macros.hpp"
#include "openbot_bridge/common_msgs/sensor_msgs.pb.h"
#include "openbot/bridge/common/grpc/grpc_client.hpp"
#include "openbot/bridge/proto/grpc_opionts.pb.h"

namespace openbot {
namespace bridge { 
namespace compontents { 

class GrpcComponent final :
    public apollo::cyber::Component<> 
{
public:
    GrpcComponent() = default;
    ~GrpcComponent();

    bool Init() override;

private:

    /**
     * @brief Main thread running
     * 
     */
    void Run();


    void HandleSensorImageMessages(const std::shared_ptr<::openbot_bridge::common_msgs::Image>& msgs);

    /**
     * @brief Handle global map
     * 
     * @param msgs 
     */
    void HandlePointCloudMessages(const std::shared_ptr<::openbot_bridge::common_msgs::PointCloud>& msgs);


    // cyber node
    std::shared_ptr<apollo::cyber::Reader<::openbot_bridge::common_msgs::Image>> sensor_image_reader_{nullptr};
    std::shared_ptr<apollo::cyber::Reader<::openbot_bridge::common_msgs::PointCloud>> pointcloud_reader_{nullptr};

    uint32_t spin_rate_ = 200;
    std::future<void> async_result_;
    std::atomic<bool> running_ = {false};

    grpc::GrpcClientImpl::SharedPtr grpc_client_{nullptr};
    std::shared_ptr<::grpc::Channel> channel_{nullptr};

    // config
    std::shared_ptr<::openbot::bridge::grpc::GRPCConfig> grpc_config_{nullptr};
};

CYBER_REGISTER_COMPONENT(GrpcComponent)

}  // namespace compontents
}  // namespace planning 
}  // namespace openbot
