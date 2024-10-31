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

#include <algorithm>
#include <iostream>
#include <vector>

// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/point_cloud.h>

#include "openbot/map/mockamap/maps.hpp"
#include "openbot/common/msgs/sensor_msgs.hpp"

namespace openbot {
namespace map { 
namespace mockamap { 

// void optimizeMap(mocka::Maps::BasicInfo& in)
// {
//   std::vector<int>* temp = new std::vector<int>;

//   pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//   cloud->width  = in.cloud->width;
//   cloud->height = in.cloud->height;
//   cloud->points.resize(cloud->width * cloud->height);

//   for (int i = 0; i < cloud->width; i++)
//   {
//     cloud->points[i].x = in.cloud->points[i].x;
//     cloud->points[i].y = in.cloud->points[i].y;
//     cloud->points[i].z = in.cloud->points[i].z;
//   }

//   kdtree.setInputCloud(cloud);
//   double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

//   for (int i = 0; i < cloud->width; i++)
//   {
//     std::vector<int>   pointIdxRadiusSearch;
//     std::vector<float> pointRadiusSquaredDistance;

//     if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
//                             pointRadiusSquaredDistance) >= 27)
//     {
//       temp->push_back(i);
//     }
//   }
//   for (int i = temp->size() - 1; i >= 0; i--)
//   {
//     in.cloud->points.erase(in.cloud->points.begin() +
//                            temp->at(i)); // erasing the enclosed points
//   }
//   in.cloud->width -= temp->size();

//   pcl::toROSMsg(*in.cloud, *in.output);
//   in.output->header.frame_id = "odom";
//   // ROS_INFO("finish: number of points after optimization %d", in.cloud->width);
//   delete temp;
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "mockamap");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_private("~");

//   ros::Publisher pcl_pub =
//     nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   sensor_msgs::PointCloud2       output;
//   // Fill in the cloud data

//   int seed;

//   int sizeX;
//   int sizeY;
//   int sizeZ;

//   double scale;
//   double update_freq;

//   int type;

//   nh_private.param("seed", seed, 4546);
//   nh_private.param("update_freq", update_freq, 1.0);
//   nh_private.param("resolution", scale, 0.38);
//   nh_private.param("x_length", sizeX, 100);
//   nh_private.param("y_length", sizeY, 100);
//   nh_private.param("z_length", sizeZ, 10);

//   nh_private.param("type", type, 1);

//   scale = 1 / scale;
//   sizeX = sizeX * scale;
//   sizeY = sizeY * scale;
//   sizeZ = sizeZ * scale;

//   mocka::Maps::BasicInfo info;
//   info.nh_private = &nh_private;
//   info.sizeX      = sizeX;
//   info.sizeY      = sizeY;
//   info.sizeZ      = sizeZ;
//   info.seed       = seed;
//   info.scale      = scale;
//   info.output     = &output;
//   info.cloud      = &cloud;

//   mocka::Maps map;
//   map.setInfo(info);
//   map.generate(type);

//   //  optimizeMap(info);

//   //! @note publish loop
//   ros::Rate loop_rate(update_freq);
//   while (ros::ok())
//   {
//     pcl_pub.publish(output);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }

}  // namespace mockamap
}  // namespace map
}  // namespace openbot