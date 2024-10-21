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


#include "gflags/gflags.h"
#include "glog/logging.h"

#include "cyber/examples/proto/examples.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Chatter;

namespace openbot {
namespace {

void Run() 
{
  LOG(INFO) << "Openbot starting !!! ";
}

}  // namespace
}  // namespace openbot

int main(int argc, char** argv) 
{
  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  // CHECK(!FLAGS_configuration_directory.empty())
  //     << "-configuration_directory is missing.";
  // CHECK(!FLAGS_configuration_basename.empty())
  //     << "-configuration_basename is missing.";

  // openbot_ros::ScopedRosLogSink ros_log_sink;
  openbot::Run();
  google::ShutdownGoogleLogging();
  return 0;
}
