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

#include "openbot/bridge/components/websocket_component.hpp"
#include "openbot/common/proto/geometry_msgs.pb.h"
#include "openbot/common/utils/json_util.hpp"
#include "openbot/common/utils/logging.hpp"
#include "nlohmann/json.hpp"

namespace openbot {
namespace bridge { 
namespace compontents { 

bool WebSocketComponent::Init()
{
    websocket_config_ = std::make_shared<http::WebsocketConfig>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, websocket_config_.get())) {
      return false;
    }
    LOG(INFO) << "Websocket config: " << websocket_config_->DebugString();
    websocket_client_ = std::make_shared<http::WebsocketClient>(websocket_config_->host(), websocket_config_->port());
    async_result_ = apollo::cyber::Async(&WebSocketComponent::Run, this);
    return true;
}

void WebSocketComponent::Run()
{
    running_.exchange(true);
    while (!apollo::cyber::IsShutdown()) {
        if (!websocket_client_->IsConnect()) {
            bool connect_success = websocket_client_->Connect();
            if (!connect_success) {
                apollo::cyber::SleepFor(std::chrono::seconds(5));
                continue;
            }
        }
  

        common::geometry_msgs::Twist twist;
        twist.mutable_linear()->set_x(1);
        twist.mutable_linear()->set_y(2);
        twist.mutable_linear()->set_z(3);
        twist.mutable_angular()->set_x(1);
        twist.mutable_angular()->set_y(2);
        twist.mutable_angular()->set_z(3);

        auto json = common::utils::JsonUtil::ProtoToJson(twist);
        LOG(INFO) << "Twist: " << json.dump();

        nlohmann::json ros_msgs;
        ros_msgs["op"] = "publish";
        ros_msgs["topic"] = "/openbot/twist";
        ros_msgs["msg"] = json;

        LOG(INFO) << "ros_msgs: " << ros_msgs.dump();

        websocket_client_->Send(ros_msgs.dump());
        apollo::cyber::SleepFor(std::chrono::seconds(1));
    }
}

WebSocketComponent::~WebSocketComponent()
{
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

}  // namespace compontents
}  // namespace planning 
}  // namespace openbot