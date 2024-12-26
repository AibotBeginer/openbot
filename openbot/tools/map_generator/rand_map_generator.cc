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

#include "openbot/tools/map_generator/rand_map_generator.hpp"

#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "glog/logging.h"

namespace openbot {
namespace tools {
namespace map_generator { 

MapGenerator::MapGenerator(const Maps::BasicInfo& option)
    : basic_option_(option)
{
    map_ = std::make_shared<Maps>();
    map_->setInfo(basic_option_);
}

MapGenerator::MapGenerator(const MapOption& option)
{
    map_ = std::make_shared<Maps>(option);
}


MapGenerator::~MapGenerator()
{

}

bool MapGenerator::Generate(::openbot_bridge::common_msgs::PointCloud2& point_cloud)
{
    map_->generate();
    if (map_->cloud() == nullptr) {
        return false;
    }

    point_cloud = *map_->cloud();
    point_cloud.mutable_header()->set_frame_id("map");
    // point_cloud.mutable_header()->stamp = common::builtin_interfaces::Time::now();
    return true;
}

}  // namespace map_generator
}  // namespace tools
}  // namespace openbot


int main(int argc, char** argv)
{
    return 0;
}