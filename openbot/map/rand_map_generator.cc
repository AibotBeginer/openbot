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

#include "openbot/map/rand_map_generator.hpp"

#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "glog/logging.h"

namespace openbot {
namespace map {

MapGenerator::MapGenerator(const mockamap::Maps::BasicInfo& option)
    : basic_option_(option)
{
    map_ = std::make_shared<mockamap::Maps>();
    map_->setInfo(basic_option_);
}

MapGenerator::MapGenerator(const mockamap::MapOption& option)
{
    map_ = std::make_shared<mockamap::Maps>(option);
}


MapGenerator::~MapGenerator()
{

}

bool MapGenerator::Generate(common::sensor_msgs::PointCloud2& point_cloud)
{
    map_->generate();
    if (map_->cloud() == nullptr) {
        return false;
    }

    point_cloud = *map_->cloud();
    point_cloud.header.frame_id = "map";
    point_cloud.header.stamp = common::builtin_interfaces::Time::now();
    return true;
}

}  // namespace map
}  // namespace openbot