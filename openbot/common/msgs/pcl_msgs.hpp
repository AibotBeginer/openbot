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

#ifndef OPENBOT_COMMON_MSGS_PCL_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_PCL_MSGS_HPP_

#include <string>
#include <vector>

#include "openbot/common/proto/std_msgs/header.pb.h"
#include "openbot/common/proto/pcl_msgs/vertices.pb.h"
#include "openbot/common/proto/pcl_msgs/polygon_mesh.pb.h"
#include "openbot/common/proto/pcl_msgs/point_indices.pb.h"
#include "openbot/common/proto/pcl_msgs/model_coefficients.pb.h"
#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/msgs/sensor_msgs.hpp"

namespace openbot {
namespace common {
namespace pcl_msgs {

struct Vertices
{
    // List of point indices
    std::vector<uint32> vertices;
};

struct PolygonMesh
{
    // Separate header for the polygonal surface
    std_msgs::Header header;

    // Vertices of the mesh as a point cloud
    sensor_msgs::PointCloud2 cloud;

    // List of polygons
    std::vector<Vertices> polygons;
};

struct PointIndices 
{
    std_msgs::Header header;
    std::vector<int32> indices;    
};

struct ModelCoefficients 
{
    std_msgs::Header header;
    std::vector<float> values;
};

// Vertices
openbot::common::proto::pcl_msgs::Vertices ToProto(const Vertices& data);
Vertices FromProto(const openbot::common::proto::pcl_msgs::Vertices& proto);

// PolygonMesh
openbot::common::proto::pcl_msgs::PolygonMesh ToProto(const PolygonMesh& data);
PolygonMesh FromProto(const openbot::common::proto::pcl_msgs::PolygonMesh& proto);

// PointIndices
openbot::common::proto::pcl_msgs::PointIndices ToProto(const PointIndices& data);
PointIndices FromProto(const openbot::common::proto::pcl_msgs::PointIndices& proto);

// ModelCoefficients
openbot::common::proto::pcl_msgs::ModelCoefficients ToProto(const ModelCoefficients& data);
ModelCoefficients FromProto(const openbot::common::proto::pcl_msgs::ModelCoefficients& proto);

}  // namespace pcl_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_PCL_MSGS_HPP_
