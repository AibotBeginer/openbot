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

#include "openbot/common/utils/logging.hpp"
#include "openbot/common/utils/version.hpp"

namespace openbot {
namespace {

void ShowVersion() 
{
  std::cout << common::utils::StringPrintf(
    "%s -- Openbot for robotics develop everyone \n(%s)",
                   common::utils::GetVersionInfo().c_str(),
                   common::utils::GetBuildInfo().c_str())
            << std::endl
            << std::endl;
}

void Run() 
{
    ShowVersion();
    LOG(INFO) << "Openbot starting !!! ";
}

}  // namespace
}  // namespace openbot

int main(int argc, char* argv[]) 
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
    return EXIT_SUCCESS;
}