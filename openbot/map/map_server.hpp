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

#include <string>

#include "openbot/common/macros.hpp"
#include "openbot/map/costmap.hpp"

namespace openbot {
namespace map { 

class MapServer
{
public:
     /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(MapServer);

    /**
     * @brief A constructor for openbot::map::MapServer
     * @param options Additional options to control creation of the node.
     */
    explicit MapServer();

    /**
     * @brief Destructor for openbot::map::MapServer
     */
    ~MapServer();

    /**
     * @brief Load matterport3d datasets
     */
    bool LoadPointClouds(const std::string& path, openbot_bridge::sensor_msgs::PointCloud& clouds);

private:
    // costmap
    Costmap::SharedPtr costmap_{nullptr};
};

}  // namespace map
}  // namespace openbot
