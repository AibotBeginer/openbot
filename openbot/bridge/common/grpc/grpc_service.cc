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

#include "openbot/bridge/common/grpc/grpc_service.hpp"

#include <chrono>

namespace openbot {
namespace bridge { 
namespace grpc {

GrpcServerImpl::GrpcServerImpl()
    : node_(::apollo::cyber::CreateNode("grpc_server")) 
{
    LOG(INFO) << "create grpc_server node";
    CHECK(!!node_);
    LOG(INFO) << "GrpcServerImpl initial success";
    init_flag_ = true;
}

GrpcServerImpl::~GrpcServerImpl() {};

::grpc::Status GrpcServerImpl::PublishImageSennorMessages(
    ::grpc::ServerContext* context, 
    const ::openbot_bridge::sensor_msgs::Image* request, 
    google::protobuf::Empty* response)
{    
    LOG(INFO) << "grpc call success !<_>!";
    return ::grpc::Status::OK;
}

}  // namespace grpc 
}  // namespace bridge 
}  // namespace openbot
