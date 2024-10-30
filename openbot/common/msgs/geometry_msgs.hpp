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

#ifndef OPENBOT_COMMON_MSGS_GEOMETRY_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_GEOMETRY_MSGS_HPP_

#include <vector>
#include <string>

#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/proto/geometry_msgs/accel.pb.h"  
#include "openbot/common/proto/geometry_msgs/accel_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/accel_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/accel_with_covariance_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/inertia.pb.h"  
#include "openbot/common/proto/geometry_msgs/inertia_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/point.pb.h"  
#include "openbot/common/proto/geometry_msgs/point32.pb.h"  
#include "openbot/common/proto/geometry_msgs/point_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/polygon.pb.h"  
#include "openbot/common/proto/geometry_msgs/polygon_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose.pb.h"  
#include "openbot/common/proto/geometry_msgs/pose_2d.pb.h"  
#include "openbot/common/proto/geometry_msgs/pose_array.pb.h"  
#include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/pose_with_covariance_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/quaternion.pb.h"  
#include "openbot/common/proto/geometry_msgs/quaternion_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/transform.pb.h"  
#include "openbot/common/proto/geometry_msgs/transform_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/twist.pb.h"  
#include "openbot/common/proto/geometry_msgs/twist_stamped.pb.h"  
#include "openbot/common/proto/geometry_msgs/twist_with_covariance.pb.h"
#include "openbot/common/proto/geometry_msgs/twist_with_covariance_stamped.pb.h"
#include "openbot/common/proto/geometry_msgs/vector3.pb.h"  
#include "openbot/common/proto/geometry_msgs/vector3_stamped.pb.h" 
#include "openbot/common/proto/geometry_msgs/velocity_stamped.pb.h" 

namespace openbot {
namespace common {
namespace geometry_msgs {

struct Vector3 
{
    float x;
    float y;
    float z;
};

struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

struct Accel 
{
    Vector3 linear;
    Vector3 angular;
};

struct AccelStamped 
{
    std_msgs::Header header;
    Accel accel;
};

struct AccelWithCovariance
{
    Accel accel;
    std::vector<float> covariance;
};

struct AccelWithCovarianceStamped
{
    std_msgs::Header header;
    AccelWithCovariance accel;
};

struct Inertia
{
    // Mass [kg]
    float m;

    // Center of mass [m]
    Vector3 com;

    // Inertia Tensor [kg-m^2]
    //     | ixx ixy ixz |
    // I = | ixy iyy iyz |
    //     | ixz iyz izz |
    float ixx;
    float ixy;
    float ixz;
    float iyy;
    float iyz;
    float izz ;
};

struct InertiaStamped
{
    std_msgs::Header header;
    Inertia inertia;
};

struct Point
{
    double x;
    double y;
    double z;
};

struct Point32
{
    float x;
    float y;
    float z;
};

struct PointStamped
{
    std_msgs::Header header;
    Point point;
};

struct Polygon
{
    std::vector<Point32> points;
};

struct PolygonStamped
{
    std_msgs::Header header;
    Polygon polygon;
};

struct Pose
{
    Point position;
    Quaternion orientation;
};

struct Pose2D
{
    double x;
    double y;
    double theta;
};

struct PoseArray
{
    std_msgs::Header header;
    std::vector<Pose> poses;
};

struct PoseStamped
{
    std_msgs::Header header;
    Pose pose;
};

struct PoseWithCovariance
{
    Pose pose;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    std::vector<double> covariance;
};

struct PoseWithCovarianceStamped 
{
    std_msgs::Header header;
    PoseWithCovariance pose;
};


struct QuaternionStamped
{
    std_msgs::Header header;
    Quaternion quaternion;
};

struct Transform
{
    Vector3 translation;
    Quaternion rotation;
};

struct TransformStamped
{
    // The frame id in the header is used as the reference frame of this transform.
    std_msgs::Header header;

    // The frame id of the child frame to which this transform points.
    std::string child_frame_id;

    // Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
    Transform transform;
};

struct Twist
{
    Vector3 linear;
    Vector3 angular;
};

struct TwistStamped
{
    std_msgs::Header header;
    Twist twist;
};

struct TwistWithCovariance
{
    Twist twist;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    std::vector<double> covariance;
};

struct TwistWithCovarianceStamped
{
    std_msgs::Header header;
    TwistWithCovariance twist;
};

struct Vector3Stamped
{
    // Note that this follows vector semantics with it always anchored at the origin,
    // so the rotational elements of a transform are the only parts applied when transforming.
    std_msgs::Header header;
    Vector3 vector;
};

struct VelocityStamped
{
    // This expresses the timestamped velocity vector of a frame 'body_frame_id' in the reference frame 'reference_frame_id' expressed from arbitrary observation frame 'header.frame_id'.
    // - If the 'body_frame_id' and 'header.frame_id' are identical, the velocity is observed and defined in the local coordinates system of the body
    //   which is the usual use-case in mobile robotics and is also known as a body twist.
    std_msgs::Header header;
    std::string body_frame_id;
    std::string reference_frame_id;
    Twist velocity;
};

struct Wrench
{
    Vector3 force;
    Vector3 torque;
};

struct WrenchStamped
{
    std_msgs::Header header;
    Wrench wrench;
};


// Accel.
proto::geometry_msgs::Accel ToProto(const Accel& data);
Accel FromProto(const proto::geometry_msgs::Accel& proto);

// AccelStamped
proto::geometry_msgs::AccelStamped ToProto(const AccelStamped& data);
AccelStamped FromProto(const proto::geometry_msgs::AccelStamped& proto);

// AccelWithCovariance
proto::geometry_msgs::AccelWithCovariance ToProto(const AccelWithCovariance& data);
AccelWithCovariance FromProto(const proto::geometry_msgs::AccelWithCovariance& proto);

// AccelWithCovarianceStamped
proto::geometry_msgs::AccelWithCovarianceStamped ToProto(const AccelWithCovarianceStamped& data);
AccelWithCovarianceStamped FromProto(const proto::geometry_msgs::AccelWithCovarianceStamped& proto);

// Inertia
proto::geometry_msgs::Inertia ToProto(const Inertia& data);
Inertia FromProto(const proto::geometry_msgs::Inertia& proto);

// InertiaStamped
proto::geometry_msgs::InertiaStamped ToProto(const InertiaStamped& data);
InertiaStamped FromProto(const proto::geometry_msgs::InertiaStamped& proto);

// Point
proto::geometry_msgs::Point ToProto(const Point& data);
Point FromProto(const proto::geometry_msgs::Point& proto);

// Point32
proto::geometry_msgs::Point32 ToProto(const Point32& data);
Point32 FromProto(const proto::geometry_msgs::Point32& proto);

// PointStamped
proto::geometry_msgs::PointStamped ToProto(const PointStamped& data);
PointStamped FromProto(const proto::geometry_msgs::PointStamped& proto);

// Pose
proto::geometry_msgs::Pose ToProto(const Pose& data);
Pose FromProto(const proto::geometry_msgs::Pose& proto);

// PoseStamped
proto::geometry_msgs::PoseStamped ToProto(const PoseStamped& data);
PoseStamped FromProto(const proto::geometry_msgs::PoseStamped& proto);

// Quaternion
proto::geometry_msgs::Quaternion ToProto(const Quaternion& data);
Quaternion FromProto(const proto::geometry_msgs::Quaternion& proto);

// Vector3
proto::geometry_msgs::Vector3 ToProto(const Vector3& data);
Vector3 FromProto(const proto::geometry_msgs::Vector3& proto);


}  // namespace geometry_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_GEOMETRY_MSGS_HPP_
