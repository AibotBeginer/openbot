#include "openbot/common/msgs/geometry_msgs.hpp"

namespace openbot {
namespace common {
namespace geometry_msgs {

proto::geometry_msgs::Accel ToProto(const Accel& data)
{
    proto::geometry_msgs::Accel proto;
    *proto.mutable_linear() = ToProto(data.linear);
    *proto.mutable_angular() = ToProto(data.angular);
    return proto;
}

Accel FromProto(const proto::geometry_msgs::Accel& proto)
{
    return Accel {
        FromProto(proto.linear()),
        FromProto(proto.angular())
    };
}

proto::geometry_msgs::AccelStamped ToProto(const AccelStamped& data)
{
    proto::geometry_msgs::AccelStamped proto;
    *proto.mutable_header() = ToProto(data.header);
    *proto.mutable_accel() = ToProto(data.accel);
    return proto;
}

AccelStamped FromProto(const proto::geometry_msgs::AccelStamped& proto)
{
    return AccelStamped {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.accel())
    };
}

proto::geometry_msgs::AccelWithCovariance ToProto(const AccelWithCovariance& data)
{
    proto::geometry_msgs::AccelWithCovariance proto;
    *proto.mutable_accel() = ToProto(data.accel);
    for (int i = 0; i < data.covariance.size(); ++i) {
        proto.set_covariance(i, data.covariance[i]);
    }
    return proto;
}

AccelWithCovariance FromProto(const proto::geometry_msgs::AccelWithCovariance& proto)
{
    AccelWithCovariance data;
    data.accel = FromProto(proto.accel());
    for (int i = 0; i < proto.covariance_size(); ++i) {
        data.covariance.push_back(proto.covariance(i));
    }
    return data;
}

proto::geometry_msgs::AccelWithCovarianceStamped ToProto(const AccelWithCovarianceStamped& data)
{
    proto::geometry_msgs::AccelWithCovarianceStamped proto;
    *proto.mutable_header() = ToProto(data.header);
    *proto.mutable_accel() = ToProto(data.accel);
    return proto;
}

AccelWithCovarianceStamped FromProto(const proto::geometry_msgs::AccelWithCovarianceStamped& proto)
{
    return AccelWithCovarianceStamped {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.accel())
    };
}

proto::geometry_msgs::Inertia ToProto(const Inertia& data)
{
    proto::geometry_msgs::Inertia proto;
    proto.set_m(data.m);
    *proto.mutable_com() = ToProto(data.com);
    proto.set_ixx(data.ixx);
    proto.set_ixy(data.ixy);
    proto.set_ixz(data.ixz);
    proto.set_iyy(data.iyy);
    proto.set_iyz(data.iyz);
    proto.set_izz(data.izz);
    return proto;
}

Inertia FromProto(const proto::geometry_msgs::Inertia& proto)
{
    return Inertia {
        proto.m(),
        FromProto(proto.com()),
        proto.ixx(),
        proto.ixy(),
        proto.ixz(),
        proto.iyy(),
        proto.iyz(),
        proto.izz()
    };
}

proto::geometry_msgs::InertiaStamped ToProto(const InertiaStamped& data)
{
    proto::geometry_msgs::InertiaStamped proto;
    *proto.mutable_header() = ToProto(data.header);
    *proto.mutable_inertia() = ToProto(data.inertia);
    return proto;
}

InertiaStamped FromProto(const proto::geometry_msgs::InertiaStamped& proto)
{
    return InertiaStamped {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.inertia())
    };
}

proto::geometry_msgs::Point ToProto(const Point& data)
{
    proto::geometry_msgs::Point proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

Point FromProto(const proto::geometry_msgs::Point& proto)
{
    return Point{
        proto.x(),
        proto.y(),
        proto.z()
    };
}

proto::geometry_msgs::Point32 ToProto(const Point32& data)
{
    proto::geometry_msgs::Point32 proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

Point32 FromProto(const proto::geometry_msgs::Point32& proto)
{
    return Point32 {
        proto.x(),
        proto.y(),
        proto.z()
    };
}

proto::geometry_msgs::PointStamped ToProto(const PointStamped& data)
{
    proto::geometry_msgs::PointStamped proto;
    *proto.mutable_header() = ToProto(data.header);
    *proto.mutable_point() = ToProto(data.point);
    return proto;
}

PointStamped FromProto(const proto::geometry_msgs::PointStamped& proto)
{
    return PointStamped {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.point())
    };
}

proto::geometry_msgs::Pose ToProto(const Pose& data)
{
    proto::geometry_msgs::Pose proto;
    *proto.mutable_position() = ToProto(data.position);
    *proto.mutable_orientation() = ToProto(data.orientation);
    return proto;
}

Pose FromProto(const proto::geometry_msgs::Pose& proto)
{
    return Pose {
        FromProto(proto.position()),
        FromProto(proto.orientation())
    };
}

proto::geometry_msgs::PoseStamped ToProto(const PoseStamped& data)
{
    proto::geometry_msgs::PoseStamped proto;
    *proto.mutable_header() = ToProto(data.header);
    *proto.mutable_pose() = ToProto(data.pose);
    return proto;
}

PoseStamped FromProto(const proto::geometry_msgs::PoseStamped& proto)
{
    return PoseStamped {
        std_msgs::FromProto(proto.header()),
        FromProto(proto.pose())
    };
}

proto::geometry_msgs::Quaternion ToProto(const Quaternion& data)
{
    proto::geometry_msgs::Quaternion proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    proto.set_w(data.w);
    return proto;
}

Quaternion FromProto(const proto::geometry_msgs::Quaternion& proto)
{
    return Quaternion {
        proto.x(),
        proto.y(),
        proto.z(),
        proto.w()
    };
}

proto::geometry_msgs::Vector3 ToProto(const Vector3& data)
{
    proto::geometry_msgs::Vector3 proto;
    proto.set_x(data.x);
    proto.set_y(data.y);
    proto.set_z(data.z);
    return proto;
}

Vector3 FromProto(const proto::geometry_msgs::Vector3& proto)
{
    return Vector3{
        proto.x(),
        proto.y(),
        proto.z()
    };
}

}  // namespace geometry_msgs
}  // namespace common
}  // namespace openbot