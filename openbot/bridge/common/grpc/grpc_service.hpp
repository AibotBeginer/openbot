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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <grpc++/grpc++.h>

#include "cyber/cyber.h"


#include "openbot_bridge/common_msgs/sensor_msgs.pb.h"
#include "openbot_bridge/service_msgs/sensor_service.pb.h"
#include "openbot_bridge/service_msgs/sensor_service.grpc.pb.h"



namespace openbot {
namespace bridge { 
namespace grpc {

class GrpcServerImpl final : public ::openbot_bridge::service_msgs::SensorService::Service 
{
public:
    /* construct function
    */
    GrpcServerImpl();
    ~GrpcServerImpl()override; 

    bool InitFlag() { return init_flag_; }

    ::grpc::Status PublishImageSennorMessages(::grpc::ServerContext* context, 
        const ::openbot_bridge::common_msgs::Image* request, google::protobuf::Empty* response) override;

    private:
    bool init_flag_ = false;
    std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
};

}  // namespace grpc 
}  // namespace bridge 
}  // namespace openbot
