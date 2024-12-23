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

#include <vector>
#include <string>

#include "openbot/common/macros.hpp"
#include "openbot/common/io/msgs.hpp"

#include "openbot/tools/map_generator/mockamap/maps.hpp"
#include "openbot/tools/map_generator/mockamap/map_opionts.hpp"

namespace openbot {
namespace tools {
namespace map_generator { 
namespace mockamap { 

class MapGenerator
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(MapGenerator);

    MapGenerator(const mockamap::Maps::BasicInfo& option);

    MapGenerator(const mockamap::MapOption& option);

    ~MapGenerator();

    bool Generate(::openbot_bridge::common_msgs::PointCloud2& point_cloud2);

private:

    mockamap::Maps::SharedPtr map_{nullptr};
    mockamap::Maps::BasicInfo basic_option_;
    mockamap::MapOption option_;
};

}  // namespace mockamap
}  // namespace map_generator
}  // namespace tools
}  // namespace openbot
