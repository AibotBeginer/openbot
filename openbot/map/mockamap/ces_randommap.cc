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

#include "openbot/map/mockamap/ces_randommap.hpp"
#include "openbot/map/mockamap/maps.hpp"

#include <random>
#include <sys/time.h>
#include <time.h>

#include "glog/logging.h"

namespace openbot {
namespace map {
namespace mockamap { 

CesRandomMap::CesRandomMap(const Option& option)
{

}

bool CesRandomMap::FixedMapGenerate(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  cloud_map_.points.clear();
  obstacles_list_.push_back(std::make_pair(ObsPos(-7.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(-1.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(10.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(16.0, 1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(-4.0, -1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(13.0, -1.0, 0.0), ObsSize(1.0, 3.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(5.0, 2.5, 0.0),   ObsSize(30.0, 1.0, 5.0)));
  obstacles_list_.push_back(std::make_pair(ObsPos(5.0, -2.5, 0.0),  ObsSize(30.0, 1.0, 5.0)));

  int num_total_obs = obstacles_list_.size();
  pcl::PointXYZ pt_insert;

  for (int i = 0; i < num_total_obs; i++)
  {
    double x, y, z;
    double lx, ly, lz;
    x  = (obstacles_list_[i].first)[0];
    y  = (obstacles_list_[i].first)[1];
    z  = (obstacles_list_[i].first)[2];
    lx = (obstacles_list_[i].second)[0];
    ly = (obstacles_list_[i].second)[1];
    lz = (obstacles_list_[i].second)[2];

    int num_mesh_x = std::ceil(lx / resolution_);
    int num_mesh_y = std::ceil(ly / resolution_);
    int num_mesh_z = std::ceil(lz / resolution_);

    int left_x, right_x, left_y, right_y, left_z, right_z;
    left_x  = -num_mesh_x / 2;
    right_x = num_mesh_x / 2;
    left_y  = -num_mesh_y / 2;
    right_y = num_mesh_y / 2;
    left_z  = 0;
    right_z = num_mesh_z;

    for (int r = left_x; r < right_x; r++) 
    {
      for (int s = left_y; s < right_y; s++)
      {
        for (int t = left_z; t < right_z; t++)
        {
          if ((r - left_x) * (r - right_x + 1) * (s - left_y) * (s - right_y + 1) * (t - left_z) * (t - right_z + 1) == 0)
          {
            pt_insert.x = x + r * resolution_;
            pt_insert.y = y + s * resolution_;
            pt_insert.z = z + t * resolution_;
            cloud_map_.points.push_back(pt_insert);
          }
        }
      }
    }
  }

  cloud_map_.width    = cloud_map_.points.size();
  cloud_map_.height   = 1;
  cloud_map_.is_dense = true;

  initialized_finished_ = true;
  LOG(WARNING) << "Finished generate random map, map size: " << cloud_map_.size();
  kdtree_local_map_.setInputCloud(cloud_map_.makeShared());
  return true;
}

bool CesRandomMap::GetPointCloud2Data(common::sensor_msgs::PointCloud2& point_cloud)
{
  if (!initialized_finished_) {
    LOG(WARNING) << "Random map not initialize finished.";
    return false;
  }

  ::openbot::common::pcl::toROSMsg(cloud_map_, global_map_pcd_);
  global_map_pcd_.header.frame_id = "map";
  point_cloud = global_map_pcd_;
  return true;
}

bool CesRandomMap::GetSensedPoints(const pcl::PointXYZ& current_point, 
  common::sensor_msgs::PointCloud2& point_cloud)
{
  if (!initialized_finished_ ) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> local_map;
  std::vector<int> point_idx_radius_search;
  std::vector<float> point_radius_squared_distance;

  point_idx_radius_search.clear();
  point_radius_squared_distance.clear();
  double sensing_range;

  pcl::PointXYZ pt_in_noflation;
  if (kdtree_local_map_.radiusSearch(current_point, sensing_range, 
    point_idx_radius_search, point_radius_squared_distance) <= 0) {
    return false;
  }

  for (size_t i = 0; i < point_idx_radius_search.size(); ++i)
  {
    pt_in_noflation = cloud_map_.points[point_idx_radius_search[i]];
    local_map.points.push_back(pt_in_noflation);
  }

  pcl::PointXYZ pt_fix;
  pt_fix.x = current_point.x;
  pt_fix.y = current_point.y;
  pt_fix.z = 0.0;
  local_map.points.push_back(pt_fix);

  local_map.width    = local_map.points.size();
  local_map.height   = 1;
  local_map.is_dense = true;

  ::openbot::common::pcl::toROSMsg(local_map, local_map_pcd_);
  local_map_pcd_.header.frame_id = "odom";
  point_cloud = local_map_pcd_;
  return true;
}

common::sensor_msgs::PointCloud2& CesRandomMap::global_map()
{
  return global_map_pcd_;
}

}  // mockamap
}  // namespace map
}  // namespace openbot



// vector<int>                        pointIdxRadiusSearch;
// vector<float>                      pointRadiusSquaredDistance;

// ros::Subscriber _map_sub;
// ros::Subscriber _odom_sub;

// deque<nav_msgs::Odometry> _odom_queue;
// vector<double>            _state;
// const size_t              _odom_queue_size = 200;
// nav_msgs::Odometry        _odom;

// double z_limit;
// double _SenseRate;
// double _sensing_range;

// // ros::Timer vis_map;
// bool map_ok    = false;
// bool _has_odom = false;

// sensor_msgs::PointCloud2       globalMap_pcd;
// sensor_msgs::PointCloud2       localMap_pcd;
// pcl::PointCloud<pcl::PointXYZ> cloudMap;
// ros::Time                      begin_time = ros::TIME_MAX;

// typedef Eigen::Vector3d ObsPos;
// typedef Eigen::Vector3d ObsSize; // x, y, height --- z
// typedef pair<ObsPos, ObsPos> Obstacle;
// std::vector<Obstacle> obstacle_list;



// void rcvOdometryCallbck(const nav_msgs::Odometry odom)
// {
//   if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
//     return;
//   _odom     = odom;
//   _has_odom = true;

//   _state = { _odom.pose.pose.position.x,
//              _odom.pose.pose.position.y,
//              _odom.pose.pose.position.z,
//              _odom.twist.twist.linear.x,
//              _odom.twist.twist.linear.y,
//              _odom.twist.twist.linear.z,
//              0.0,
//              0.0,
//              0.0 };

//   _odom_queue.push_back(odom);
//   while (_odom_queue.size() > _odom_queue_size)
//     _odom_queue.pop_front();
// }