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

#include "cyber/cyber.h"

#include "openbot/common/utils/logging.hpp"
#include "openbot/common/utils/version.hpp"

#include "openbot/bridge/common/grpc/grpc_client.hpp"

#include "absl/strings/str_format.h"

#include <gflags/gflags.h>
#include <grpcpp/grpcpp.h>
#include <grpc/impl/codegen/connectivity_state.h>


DEFINE_string(grpc_client_host, "localhost", "Server host for the service");
DEFINE_int32(grpc_client_port, 5005, "Server port for the service");

using ::openbot::bridge::grpc::GrpcClientImpl;


int main(int argc, char* argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    // init cyber framework
    apollo::cyber::Init(argv[0]);

    auto server_address = ::absl::StrFormat("%s:%d", FLAGS_grpc_client_host, FLAGS_grpc_client_port);
    LOG(INFO) << "connecting to " << server_address;
    std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
    grpc_connectivity_state state = channel->GetState(true);
    switch (state) {
        case GRPC_CHANNEL_IDLE:
            printf("Channel is idle.\n");
            break;
        case GRPC_CHANNEL_CONNECTING:
            printf("Channel is connecting.\n");
            break;
        case GRPC_CHANNEL_READY:
            printf("Channel is ready.\n");
            break;
        case GRPC_CHANNEL_TRANSIENT_FAILURE:
            printf("Channel encountered a transient failure.\n");
            break;
        case GRPC_CHANNEL_SHUTDOWN:
            printf("Channel is shutdown.\n");
            break;
        default:
            printf("Unknown channel state.\n");
            break;
    }


    std::unique_ptr<GrpcClientImpl> grpc_client (new GrpcClientImpl(channel));
    grpc_client->InitFlag();
    
    //for testing....
    
    for (int i = 0; i < 100; i++) {
        std::shared_ptr<::openbot_bridge::sensor_msgs::Image> msg(new ::openbot_bridge::sensor_msgs::Image());
        msg->set_height(100);
        msg->set_width(100);
        LOG(INFO) << "set msg: height>>" << msg->height();
	grpc_client->SendMsgToGrpc(msg);
    }
    LOG(INFO) << "======================start listener====================";

    auto send_msg_fn = std::mem_fn(&openbot::bridge::grpc::GrpcClientImpl::SendMsgToGrpc);
    
    // create grpc_client node
    auto grpc_client_node = apollo::cyber::CreateNode("grpc_client");

    // create listener
    auto listener = grpc_client_node->CreateReader<::openbot_bridge::sensor_msgs::Image>("/openbot/sensor/camera/test/image",
			  std::bind(send_msg_fn, grpc_client.get(), std::placeholders::_1));

    apollo::cyber::WaitForShutdown();
    return EXIT_SUCCESS;
}
