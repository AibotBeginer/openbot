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
#include <string>

#include <grpcpp/grpcpp.h>
#include <grpc/impl/codegen/connectivity_state.h>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"

#include "openbot/common/macros.hpp"
#include "openbot_bridge/sensor_msgs/sensor_image.pb.h"
#include "openbot/bridge/common/http/websocket_client.hpp"

namespace openbot {
namespace bridge { 
namespace compontents { 

class WebSocketComponent final :
    public apollo::cyber::Component<> 
{
public:
    WebSocketComponent() = default;
    ~WebSocketComponent() = default;

    bool Init() override;

private:
    // websocket_client
    http::WebsocketClient::SharedPtr websocket_client_{nullptr};
};

CYBER_REGISTER_COMPONENT(WebSocketComponent)

}  // namespace compontents
}  // namespace planning 
}  // namespace openbot
