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
#include <mutex>

#include <grpc++/grpc++.h>

// #include "openbot/common/proto/v2x/v2x_car_to_obu_service.grpc.pb.h"

namespace openbot {
namespace bridge { 
namespace grpc { 

class GrpcClientImpl 
{
public:
    /* construct function
    @param input car_status type msg shared ptr
    */
    explicit GrpcClientImpl(std::shared_ptr<::grpc::Channel> channel);

    ~GrpcClientImpl() {}

    bool InitFlag() { return init_flag_; }

    // /*function that send car status msg through grpc
    // @param input car_status type msg shared ptr
    // */
    // void SendMsgToGrpc(const std::shared_ptr<::openbot::common::proto::v2x::CarStatus> &msg);

    // private:
    // //  grpc service stub
    // std::unique_ptr<::openbot::common::proto::v2x::CarToObu::Stub> stub_;
    // int car_status_tv_nsec_ = 0;
    // bool init_flag_ = false;
};
}  // namespace grpc 
}  // namespace bridge 
}  // namespace openbot
