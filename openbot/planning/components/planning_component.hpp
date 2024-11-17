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

#include "openbot/common/macros.hpp"
#include "openbot/planning/planner_server.hpp"

// #include "cyber/class_loader/class_loader.h"
// #include "cyber/component/component.h"
// #include "cyber/message/raw_message.h"


#include "openbot/map/proto/grid_map.pb.h"

namespace openbot {
namespace planning { 

// class PlanningComponent final :
//     public apollo::cyber::Component<::openbot::map::proto::GridMap> 
// {
// public:
//     PlanningComponent() = default;
//     ~PlanningComponent() = default;

//     bool Init() override;
//     bool Proc(const std::shared_ptr<::openbot::map::proto::GridMap>& costmap) override;

// private:
//     PlannerServer::SharedPtr planner_server_{nullptr};
//     std::shared_ptr<apollo::cyber::Reader<::openbot::map::proto::GridMap>> grid_map_reader_;
// };

}  // namespace planning 
}  // namespace openbot
