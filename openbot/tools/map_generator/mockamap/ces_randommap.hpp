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

#ifndef OPENBOT_MAP_MOCKAMAP_CES_RANDOMMAP_HPP
#define OPENBOT_MAP_MOCKAMAP_CES_RANDOMMAP_HPP

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>

#include <Eigen/Eigen>
#include <Eigen/SVD>

#include "openbot/common/macros.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/utils/pcl_conversions.hpp"
#include "openbot/tools/map_generator/mockamap/map_opionts.hpp"

namespace openbot {
namespace tools {
namespace map_generator { 
namespace mockamap { 

using ObsPos = Eigen::Vector3d;
using ObsSize = Eigen::Vector3d ; // x, y, height --- z
using Obstacle = std::pair<Eigen::Vector3d, Eigen::Vector3d>;


class CesRandomMap
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(CesRandomMap);

    CesRandomMap(const MapOption& option);

    bool FixedMapGenerate(pcl::PointCloud<pcl::PointXYZ>& cloud);

    bool GetPointCloud2Data(common::sensor_msgs::PointCloud2& point_cloud);

    bool GetSensedPoints(const pcl::PointXYZ& current_point, common::sensor_msgs::PointCloud2& point_cloud);

    common::sensor_msgs::PointCloud2& global_map();

    double resolution() { return resolution_; }

    pcl::PointCloud<pcl::PointXYZ> cloud_map_;
    common::sensor_msgs::PointCloud2 global_map_pcd_;
    common::sensor_msgs::PointCloud2 local_map_pcd_;

    std::vector<Obstacle> obstacles_list_;

    double resolution_ = 0.1f;

    // kdtree_local_map
    pcl::search::KdTree<pcl::PointXYZ> kdtree_local_map_;

    // initialized
    bool initialized_finished_{false};
};

}  // namespace mockamap
}  // namespace map_generator
}  // namespace tools
}  // namespace openbot

