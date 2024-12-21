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

#include "openbot/map/costmap.hpp"

#include "cyber/cyber.h"

namespace openbot {
namespace map {

bool Costmap::LoadOctomapFile(const std::string& octomap_filename, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    return true;
}

bool Costmap::LoadPCDFile(const std::string& pcd_filename, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    return true;
}

bool Costmap::LoadPlyFile(const std::string& ply_filename, openbot_bridge::sensor_msgs::PointCloud& clouds)
{
    // std::vector<PlyPoint>
    auto ply_points = common::utils::ReadPly(ply_filename);
    if (ply_points.empty()) {
        return false;
    }

    auto header_time = apollo::cyber::Time::Now().ToSecond();
    clouds.mutable_header()->set_timestamp_sec(header_time);
    clouds.mutable_header()->set_frame_id("map");
    clouds.set_measurement_time(header_time);
    clouds.set_height(ply_points.size());
    clouds.set_width(1);
    clouds.set_is_dense(false);
    for (const auto& ply_point : ply_points) {
        auto point_new = clouds.add_point();
        point_new->set_x(ply_point.x);
        point_new->set_y(ply_point.y);
        point_new->set_z(ply_point.z);
    }
    return true;
}

}  // namespace map
}  // namespace openbot