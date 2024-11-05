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

#ifndef OPENBOT_MAP_MOCKAMAP_MAPS_HPP
#define OPENBOT_MAP_MOCKAMAP_MAPS_HPP

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "openbot/common/pcl_conversions.hpp"
#include "openbot/common/msgs/sensor_msgs.hpp"

namespace openbot {
namespace map { 
namespace mockamap { 

class Maps 
{
public:
  typedef struct BasicInfo 
  {
    int sizeX;
    int sizeY;
    int sizeZ;
    int seed;
    double scale;
    ::openbot::common::sensor_msgs::PointCloud2 *output;
    pcl::PointCloud<pcl::PointXYZ> *cloud;
  } BasicInfo;

  Maps();

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);
  void generate(int type);

private:
  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();

  void PCLToPointCloud2();

  BasicInfo info;
};

class MazePoint 
{
private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

}  // namespace mockamap
}  // namespace map
}  // namespace openbot

#endif // OPENBOT_MAP_MOCKAMAP_MAPS_HPP
