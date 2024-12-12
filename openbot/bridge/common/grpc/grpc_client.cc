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

#include "openbot/bridge/common/grpc/grpc_client.hpp"

#include <chrono>

#include "cyber/common/log.h"
#include "cyber/cyber.h"


namespace openbot {
namespace bridge { 
namespace grpc {


using ::openbot::common::proto::v2x::CarStatus;
using ::openbot::common::proto::v2x::UpdateStatus;

using ::grpc::Channel;
using ::grpc::ClientContext;
using ::grpc::Status;

    GrpcClientImpl::GrpcClientImpl(std::shared_ptr<Channel> channel)
        : stub_(::openbot::common::proto::v2x::CarToObu::NewStub(channel)) {
      LOG(INFO) << "GrpcClientImpl initial success";
      car_status_tv_nsec_ = (1000000000 / 10 / 2);
      init_flag_ = true;
    }

    void GrpcClientImpl::SendMsgToGrpc(const std::shared_ptr<CarStatus> &msg) {
      std::cout << "SendMsgToGrpc  >>>>>" << std::endl;

      // set timeout
      ClientContext context;
      gpr_timespec timespec;
      timespec.tv_sec = 0;
      timespec.tv_nsec = car_status_tv_nsec_;  // 80000000; // 80ms
      timespec.clock_type = GPR_TIMESPAN;
      //context.set_deadline(timespec);
      UpdateStatus response;

      // time used statistics
      auto start = std::chrono::steady_clock::now();
      Status status = stub_->PushCarStatus(&context, *msg, &response);
      if (status.ok()) {
          std::cout << "Operation succeeded." << std::endl;
      } else {
          std::cout << "Operation failed: " << status.error_message() << std::endl;
      }
      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> time_used = end - start;
      // response check: error_code 4: time out; 0: success;
      LOG(INFO) << "stub PushCarStatus Time used: " << time_used.count() * 1000 << " ms";
      return;
    }

}  // namespace grpc 
}  // namespace bridge 
}  // namespace openbot
