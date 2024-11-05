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

common::sensor_msgs::PointCloud2& CesRandomMap::global_map()
{
  return global_map_pcd_;
}

}  // mockamap
}  // namespace map
}  // namespace openbot



// using namespace std;
// using namespace mocka;

// #if MAP_OR_WORLD
// const string kFrameIdNs_ = "/map";
// #else
// const string kFrameIdNs_ = "/world";
// #endif

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
// vector<int>                        pointIdxRadiusSearch;
// vector<float>                      pointRadiusSquaredDistance;

// ros::Publisher _local_map_pub;
// ros::Publisher _local_map_inflate_pub;
// ros::Publisher _global_map_pub;

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

// int frequence_division_global = 40;



// void pubSensedPoints()
// {
//   if (!map_ok || !_has_odom)
//     return;

//   ros::Time time_bef_sensing = ros::Time::now();

//   pcl::PointCloud<pcl::PointXYZ> localMap;

//   pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
//   pointIdxRadiusSearch.clear();
//   pointRadiusSquaredDistance.clear();

//   pcl::PointXYZ ptInNoflation;

//   if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
//                                   pointIdxRadiusSearch,
//                                   pointRadiusSquaredDistance) > 0)
//   {
//     for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
//     {
//       ptInNoflation = cloudMap.points[pointIdxRadiusSearch[i]];
//       localMap.points.push_back(ptInNoflation);
//     }
//   }
//   else
//   {
//     // ROS_ERROR("[Map server] No obstacles .");
//     // cout<<searchPoint.x<<" , "<<searchPoint.y<<" , "<<searchPoint.z<<endl;
//     // return;
//   }

//   pcl::PointXYZ pt_fix;
//   pt_fix.x = _state[0];
//   pt_fix.y = _state[1];
//   pt_fix.z = 0.0;
//   localMap.points.push_back(pt_fix);

//   localMap.width    = localMap.points.size();
//   localMap.height   = 1;
//   localMap.is_dense = true;

//   pcl::toROSMsg(localMap, localMap_pcd);

//   localMap_pcd.header.frame_id = kFrameIdNs_;
//   _local_map_pub.publish(localMap_pcd);

//   ros::Time time_aft_sensing = ros::Time::now();

//   if ((time_aft_sensing - begin_time).toSec() > 5.0)
//     return;

//   frequence_division_global--;
//   if (frequence_division_global == 0)
//   {
//     pcl::toROSMsg(cloudMap, globalMap_pcd);
//     globalMap_pcd.header.frame_id = kFrameIdNs_;
//     _global_map_pub.publish(globalMap_pcd);
//     frequence_division_global = 40;
//     ROS_INFO("[SERVER]Publish one global map");
//   }
// }
