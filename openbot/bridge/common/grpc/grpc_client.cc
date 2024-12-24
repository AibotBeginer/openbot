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
#include "openbot/common/utils/logging.hpp"

#include <chrono>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace openbot {
namespace bridge { 
namespace grpc {


using ::grpc::Channel;
using ::grpc::ClientContext;
using ::grpc::Status;

GrpcClientImpl::GrpcClientImpl(std::shared_ptr<Channel> channel)
    : stub_(::openbot_bridge::service_msgs::SensorService::NewStub(channel)) 
{
  LOG(INFO) << "GrpcClientImpl initial success";
  tv_nsec_ = (1000000000 / 10 / 2);
  init_flag_ = true;
}

void GrpcClientImpl::SendMsgToGrpc(const std::shared_ptr<::openbot_bridge::common_msgs::Image>& msg) 
{
  LOG(INFO) << "SendMsgToGrpc  >>>>>";
  // set timeout
  ClientContext context;
  gpr_timespec timespec;
  timespec.tv_sec = 0;
  timespec.tv_nsec = tv_nsec_;  // 80000000; // 80ms
  timespec.clock_type = GPR_TIMESPAN;

  // time used statistics
  auto start = std::chrono::steady_clock::now();
  ::google::protobuf::Empty empty;
  Status status = stub_->PublishImageSennorMessages(&context, *msg, &empty);
  if (status.ok()) {
      std::cout << "Operation succeeded." << std::endl;
  } else {
      std::cout << "Operation failed: " << status.error_message() << std::endl;
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = end - start;
  // response check: error_code 4: time out; 0: success;
  LOG(INFO) << "stub PushCarStatus Time used: " << time_used.count() * 1000 << " ms";
}

void GrpcClientImpl::SendMsgToGrpc(const std::shared_ptr<::openbot_bridge::common_msgs::PointCloud>& msg)
{
  ClientContext context;
  ::google::protobuf::Empty empty;
  Status status = stub_->PublishPointCloudSensorMessages(&context, *msg, &empty);
  if (status.ok()) {
      LOG(INFO) << "Call publish pointCloud sennor messages operation succeeded.";
  } else {
      LOG(ERROR) << "Call publish pointCloud sennor messages operation failed: " 
                 << status.error_message();
  }
}

}  // namespace grpc 
}  // namespace bridge 
}  // namespace openbot
