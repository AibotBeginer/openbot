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
#include "openbot/common/port.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/pcl_conversions.hpp"

namespace openbot {
namespace map { 
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

    struct Option
    {
        //  box edge length, unit meter
        double resolution;
        uint32 x_length;
        uint32 y_length;
        uint32 z_length;
        uint32 type;    // 1 perlin noise 3D
                        // 2 perlin box random map
                        // 3 2d maze 
                        // 4 3d maze
        /**
        type = 1 perlin noise parameters
            complexity:    base noise frequency, large value will be complex typical 0.0 ~ 0.5
            fill:          infill persentage typical: 0.4 ~ 0.0 
            fractal:       large value will have more detail
            attenuation:   for fractal attenuation typical: 0.0 ~ 0.5

        complexity = 0.03;
        fill = 0.3;
        fractal = 1;
        attenuation = 0.1;
        */
        double complexity;
        double fill;
        double attenuation;
        uint32 fractal;

        /**
        type = 2  perlin box random map
            width_min = 0.6;
            width_max = 1.5;
            obstacle_number = 50;
        */
        double width_min;
        double width_max;
        uint32 obstacle_number;

        /**
        type = 3  2d maze 
            road_width = 0.5;
            add_wall_x = 0;
            add_wall_y = 1;
            maze_type = 1;
        */
        double road_width;
        uint32 add_wall_x;
        uint32 add_wall_y;
        uint32 maze_type;  // 1 recursive division maze

        /**
        type = 4  3d maze 
            num_nodes = 40;
            connectivity = 0.8;
            node_rad = 1;
            road_rad = 10;
        */
       uint32 num_nodes;
       double connectivity;
       uint32 node_rad;
       uint32 road_rad;
    };

    CesRandomMap(const Option& option);

    bool FixedMapGenerate(pcl::PointCloud<pcl::PointXYZ>& cloud);

    bool GetPointCloud2Data(common::sensor_msgs::PointCloud2& point_cloud);

    common::sensor_msgs::PointCloud2& global_map();

    double resolution() { return resolution_; }

private:
    Option option_;

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
}  // namespace map
}  // namespace openbot

#endif // OPENBOT_MAP_MOCKAMAP_CES_RANDOMMAP_HPP
