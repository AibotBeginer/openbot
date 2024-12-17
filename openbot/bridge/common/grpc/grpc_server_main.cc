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


#include <grpcpp/grpcpp.h>
#include <grpc++/grpc++.h>
#include <gflags/gflags.h>

#include "absl/strings/str_format.h"
#include "openbot/common/utils/logging.hpp"
#include "openbot/common/utils/version.hpp"
#include "openbot/bridge/common/grpc/grpc_service.hpp"


DEFINE_string(grpc_server_host, "localhost", "Server host for the service");
DEFINE_int32(grpc_server_port, 5005, "Server port for the service");

namespace openbot {

using ::grpc::Server;
using ::grpc::ServerBuilder;
using ::grpc::ServerContext;

using ::openbot::bridge::grpc::GrpcServerImpl;

void RunServer();  // 声明


std::mutex mutex_;
std::condition_variable condition_;
std::unique_ptr<std::thread> thread_grpc_;
bool exit_flag_ = false;
void InitialServer() {
    // thread pool
    thread_grpc_ = std::make_unique<std::thread>([](){
        ::openbot::RunServer();
    });
}

void RunServer() {
    // LOG(INFO) << "grpc server starting....";
    // std::unique_lock<std::mutex> lck(mutex_);
    // auto start = std::chrono::steady_clock::now();
    // std::string server_address = ::absl::StrFormat("%s:%d", FLAGS_grpc_server_host, FLAGS_grpc_server_port);
 
    // ServerBuilder builder;
    // // Listen on the given address without any authentication mechanism.
    // builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // // Register "service" as the instance through which we'll communicate with
    // // clients. In this case it corresponds to an *synchronous* service.
    // std::unique_ptr<GrpcServerImpl> grpc_server_(new GrpcServerImpl());
    // builder.RegisterService(grpc_server_.get());
    // // Finally assemble the server.
    // std::unique_ptr<::grpc::Server> server(builder.BuildAndStart());
    // auto end = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used = end - start;
    // LOG(INFO) << "grpc server has listening on : "
    //           << server_address << " time used : " << time_used.count();
    // LOG(INFO) << "Before wait, exit_flag_: " << exit_flag_;
    // condition_.wait(lck, [&]() { return exit_flag_; });
    // LOG(INFO) << "After wait, exit_flag_: " << exit_flag_;
}

// void StopServer() {
//     LOG(INFO) << "GRPC Server Stopping ...";
//     {
//         std::unique_lock<std::mutex> lck(mutex_);
//         exit_flag_ = true;
//     }
//     condition_.notify_all();
//     LOG(INFO) << "sigal notify ...";
//     if (!!thread_grpc_ && thread_grpc_->joinable()) {
//         thread_grpc_->join();
//     }
//     LOG(INFO) << "GRPC Stop success!!";
// }

}  // namespace openbot

int main(int argc, char* argv[])
{
    // google::ParseCommandLineFlags(&argc, &argv, true);

    // // init cyber framework
    // apollo::cyber::Init(argv[0]);
    // LOG(INFO) << "openbot::InitialServer() start==========" ;
    // openbot::InitialServer();

    // apollo::cyber::WaitForShutdown();
    // openbot::StopServer();
    return EXIT_SUCCESS;
}
