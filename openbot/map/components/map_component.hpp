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
#include <unordered_map>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"

#include "openbot/common/macros.hpp"
#include "openbot/map/proto/map_config.pb.h"
#include "openbot/map/map_server.hpp"

using openbot::map::MapConfig;

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

namespace openbot {
namespace map { 

class MapComponent final : public apollo::cyber::Component<> 
{
public:
    ~MapComponent();

    bool Init() override;

private:
    void Run();

    // cyber node
    std::shared_ptr<Writer<openbot_bridge::common_msgs::PointCloud>> map_writer_{nullptr};
    uint32_t spin_rate_ = 200;
    std::future<void> async_result_;
    std::atomic<bool> running_ = {false};

    // map_config
    std::shared_ptr<MapConfig> map_config_{nullptr};

    // map_server 
    MapServer::SharedPtr map_server_{nullptr};
};

CYBER_REGISTER_COMPONENT(MapComponent)

}  // namespace map 
}  // namespace openbot
