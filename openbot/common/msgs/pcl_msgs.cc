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

#include "openbot/common/msgs/pcl_msgs.hpp"

namespace openbot {
namespace common {
namespace pcl_msgs {

openbot::common::proto::pcl_msgs::Vertices ToProto(const Vertices& data)
{
    openbot::common::proto::pcl_msgs::Vertices proto;
    return proto;
}

Vertices FromProto(const openbot::common::proto::pcl_msgs::Vertices& proto)
{
    Vertices data;
    return data;
}

openbot::common::proto::pcl_msgs::PolygonMesh ToProto(const PolygonMesh& data)
{
    openbot::common::proto::pcl_msgs::PolygonMesh proto;
    return proto;
}

PolygonMesh FromProto(const openbot::common::proto::pcl_msgs::PolygonMesh& proto)
{
    PolygonMesh data;
    return data;
}

openbot::common::proto::pcl_msgs::PointIndices ToProto(const PointIndices& data)
{
    openbot::common::proto::pcl_msgs::PointIndices proto;
    return proto;
}

PointIndices FromProto(const openbot::common::proto::pcl_msgs::PointIndices& proto)
{
    PointIndices data;
    return data;
}

openbot::common::proto::pcl_msgs::PolygonMesh ToProto(const PolygonMesh& data)
{
    openbot::common::proto::pcl_msgs::PolygonMesh proto;
    return proto;
}

PolygonMesh FromProto(const openbot::common::proto::pcl_msgs::PolygonMesh& proto)
{
    PolygonMesh data;
    return data;
}

}  // namespace pcl_msgs
}  // namespace common
}  // namespace openbot