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

#include "openbot/common/msgs/sensor_msgs.hpp"

namespace openbot {
namespace common {
namespace sensor_msgs {

proto::sensor_msgs::ChannelFloat32 ToProto(const ChannelFloat32& data)
{
    proto::sensor_msgs::ChannelFloat32 proto;
    return proto;
}

ChannelFloat32 FromProto(const proto::sensor_msgs::ChannelFloat32& proto)
{
    ChannelFloat32 data;
    return data;
}

proto::sensor_msgs::CompressedImage ToProto(const CompressedImage& data)
{
    proto::sensor_msgs::CompressedImage proto;
    return proto;
}

CompressedImage FromProto(const proto::sensor_msgs::CompressedImage& proto)
{
    CompressedImage data;
    return data;
}

proto::sensor_msgs::Illuminance ToProto(const Illuminance& data)
{
    proto::sensor_msgs::Illuminance proto;
    return proto;
}

Illuminance FromProto(const proto::sensor_msgs::Illuminance& proto)
{
    Illuminance data;
    return data;
}

proto::sensor_msgs::Image ToProto(const Image& data)
{
    proto::sensor_msgs::Image proto;
    return proto;
}

Image FromProto(const proto::sensor_msgs::Image& proto)
{
    Image data;
    return data;
}

proto::sensor_msgs::Imu ToProto(const Imu& data)
{
    proto::sensor_msgs::Imu proto;
    return proto;
}

Imu FromProto(const proto::sensor_msgs::Imu& proto)
{
    Imu data;
    return data;
}

proto::sensor_msgs::LaserScan ToProto(const LaserScan& data)
{
    proto::sensor_msgs::LaserScan proto;
    return proto;
}

LaserScan FromProto(const proto::sensor_msgs::LaserScan& proto)
{
    LaserScan data;
    return data;
}

proto::sensor_msgs::PointField ToProto(const PointField& data)
{
    proto::sensor_msgs::PointField proto;
    return proto;
}

PointField FromProto(const proto::sensor_msgs::PointField& proto)
{
    PointField data;
    return data;
}

proto::sensor_msgs::PointCloud ToProto(const PointCloud& data)
{
    proto::sensor_msgs::PointCloud proto;
    return proto;
}

PointCloud FromProto(const proto::sensor_msgs::PointCloud& proto)
{
    PointCloud data;
    return data;
}

proto::sensor_msgs::PointCloud2 ToProto(const PointCloud2& data)
{
    proto::sensor_msgs::PointCloud2 proto;
    return proto;
}

PointCloud2 FromProto(const proto::sensor_msgs::PointCloud2& proto)
{
    PointCloud2 data;
    return data;
}

// Range
proto::sensor_msgs::Range ToProto(const Range& data)
{
    proto::sensor_msgs::Range proto;
    return proto;
}

Range FromProto(const proto::sensor_msgs::Range& proto)
{
    Range data;
    return data;
}

// RegionOfInterest
proto::sensor_msgs::RegionOfInterest ToProto(const RegionOfInterest& data)
{
    proto::sensor_msgs::RegionOfInterest proto;
    return proto;
}

RegionOfInterest FromProto(const proto::sensor_msgs::RegionOfInterest& proto)
{
    RegionOfInterest data;
    return data;
}

}  // namespace sensor_msgs
}  // namespace common
}  // namespace openbot