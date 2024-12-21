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

#ifndef OPENBOT_MAP_COSTMAP_HPP_
#define OPENBOT_MAP_COSTMAP_HPP_

#include "openbot/common/macros.hpp"
#include "openbot/common/io/msgs.hpp"
#include "openbot/common/utils/ply.hpp"
#include "openbot_bridge/sensor_msgs/pointcloud.pb.h"

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace openbot {
namespace map { 

class Costmap
{
public:
    /**
     * @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(Costmap)

    /**
     * @brief Default contructor
     */
    Costmap() = default;

    /**
     * @brief Virtual destructor
     */
    ~Costmap() = default;

    /**
     * @brief Load octomap_filename convert to `pcl::PointCloud<pcl::PointXYZ>` format cloud
     * 
     * @param octomap_filename The octomap format filename
     * @param cloud Ourput map data 
     * @return return Sccess or failed
     */
    bool LoadOctomapFile(const std::string& octomap_filename, pcl::PointCloud<pcl::PointXYZ>& cloud);

     /**
     * @brief Load pcd_filename convert to `pcl::PointCloud<pcl::PointXYZ>` format cloud
     * 
     * @param pcd_filename The pcd format filename
     * @param cloud Ourput map data 
     * @return return Sccess or failed
     */
    bool LoadPCDFile(const std::string& pcd_filename, pcl::PointCloud<pcl::PointXYZ>& cloud);

     /**
     * @brief Load ply_filename convert to `pcl::PointCloud<pcl::PointXYZ>` format cloud
     * 
     * @param ply_filename The ply format filename
     * @param cloud Ourput map data 
     * @return return Sccess or failed
     */
    bool LoadPlyFile(const std::string& ply_filename, openbot_bridge::sensor_msgs::PointCloud& clouds);

    /**
     * @brief Map resolution
     * @return Return map's resolution
     */
    double resolution();

    /**
     * @brief Map size x
     * @return Return map's size x
     */
    double size_x();

    /**
     * @brief Map size y
     * @return Return map's size y
     */
    double size_y();

    /**
     * @brief Map size z
     * @return Return map's size z
     */
    double size_z();

    /**
     * @brief Map data size(x, y, z)
     */
    common::geometry_msgs::Point size() const;

    /**
     * @brief Map orgin position
     */
    common::geometry_msgs::Point origin() const;

    /**
     * @brief Map postion cost value
     * @return  Return map's position cost value
     */
    double Cost(const double x, const double y, const double z);

    /**
     * @brief Check map data position(x, y, z) is freespace
     * @return return Sccess or failed
     */
    bool IsFreespace(const double x, const double y, const double z);

    /**
     * @brief Check map data common::geometry_msgs::Point position(x, y, z) is freespace
     * @return return Sccess or failed
     */
    template<typename T>
    bool IsFreespace(const T& pose);

    /**
     * @brief Check map pose in Boundary
     * @return return Sccess or failed
     */
    template<typename T>
    bool CheckInBoundary(const T& pose);

    /**
     * @brief Convert pcl::PointCloud to occupancy grid map (2D)
     * @return return Sccess or failed
     */
    bool ToOccupancyGridMap(common::nav_msgs::OccupancyGrid& data);
};

}  // namespace map
}  // namespace openbot

#endif  // OPENBOT_MAP_COSTMAP_HPP_
