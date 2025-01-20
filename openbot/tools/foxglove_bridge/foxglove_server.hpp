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

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>
#include <unordered_set>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include "openbot/common/macros.hpp"
#include "openbot/common/status/status.hpp"

// #include "foxglove/websocket/base64.hpp"
// #include <foxglove/websocket/server_factory.hpp>
// #include <foxglove/websocket/websocket_notls.hpp>
// #include <foxglove/websocket/websocket_server.hpp>

namespace openbot {
namespace tools {
namespace foxglove_bridge {

class FoxgloveServer
{
public:

    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(FoxgloveServer);

     /**
     * @brief A constructor for openbot::tools::foxglove_bridge::FoxgloveServer
     * @param options Additional options to control creation of the node.
     */
    FoxgloveServer();

    /**
     * @brief Destructor for openbot::tools::foxglove_bridge::FoxgloveServer
     */
    ~FoxgloveServer();

    openbot::common::Status Init();

    /**
     * @brief Start foxglove viewer
     * 
     * @return openbot::common::Status 
     */
    openbot::common::Status Start();

    /**
     * @brief Stop foxglove viewer
     * 
     */
    void Stop();

private:


};



}  // namespace foxglove_bridge
}  // namespace tools
}  // namespace openbot