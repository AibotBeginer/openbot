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

#include <filesystem>

#include "cyber/init.h"

#include "openbot/tools/foxglove_bridge/foxglove_view.hpp"

// https://github.com/foxglove/ws-protocol/blob/main/docs/spec.md

int main(int argc, char* argv[]) 
{
    // set working directory to APOLLO_RUNTIME_PATH for relative file paths
    const char* openbot_runtime_path = std::getenv("OPENBOT_RUNTIME_PATH");
    if (openbot_runtime_path != nullptr) {
        if (std::filesystem::is_directory(
                std::filesystem::status(openbot_runtime_path))) {
            std::filesystem::current_path(openbot_runtime_path);
        }
    }
    google::ParseCommandLineFlags(&argc, &argv, true);
    ::apollo::cyber::Init(argv[0]);

    auto viewer = std::make_shared<::openbot::tools::foxglove_bridge::FoxgloveViewer>();
    const bool init_success = viewer->Init().ok() && viewer->Start().ok();
    if (!init_success) {
        AERROR << "Failed to initialize foxglove view bridge server";
        return -1;
    }
    ::apollo::cyber::WaitForShutdown();
    return 0;
}