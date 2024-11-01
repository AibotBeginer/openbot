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

#ifndef OPENBOT_COMMON_MSGS_SENSOR_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_SENSOR_MSGS_HPP_

#include <vector>
#include <string>

#include "openbot/common/proto/sensor_msgs/camera_info.pb.h" 
#include "openbot/common/proto/sensor_msgs/channel_float32.pb.h" 
#include "openbot/common/proto/sensor_msgs/compressed_image.pb.h"
#include "openbot/common/proto/sensor_msgs/illuminance.pb.h"
#include "openbot/common/proto/sensor_msgs/image.pb.h"
#include "openbot/common/proto/sensor_msgs/imu.pb.h" 
#include "openbot/common/proto/sensor_msgs/laser_scan.pb.h"
#include "openbot/common/proto/sensor_msgs/point_cloud.pb.h"
#include "openbot/common/proto/sensor_msgs/point_cloud2.pb.h"
#include "openbot/common/proto/sensor_msgs/point_field.pb.h"
#include "openbot/common/proto/sensor_msgs/range.pb.h"
#include "openbot/common/proto/sensor_msgs/region_of_interest.pb.h"

#include "openbot/common/msgs/std_msgs.hpp"
#include "openbot/common/msgs/geometry_msgs.hpp"
#include "openbot/common/port.hpp"

namespace openbot {
namespace common {
namespace sensor_msgs {

struct ChannelFloat32
{
    // The channel name should give semantics of the channel (e.g.
    // "intensity" instead of "value").
    std::string name;

    // The values array should be 1-1 with the elements of the associated
    // PointCloud.
    std::vector<float> values;
};

struct CompressedImage
{
    std_msgs::Header header;    // Header timestamp should be acquisition time of image
                                // Header frame_id should be optical frame of camera
                                // origin of frame should be optical center of cameara
                                // +x should point to the right in the image
                                // +y should point down in the image
                                // +z should point into to plane of the image

    std::string format;         // Specifies the format of the data
                                //   Acceptable values:
                                //     jpeg, png, tiff

    std::vector<uint32> data;   // Compressed image buffer
};

struct Illuminance
{
    std_msgs::Header header;    // timestamp is the time the illuminance was measured
                                // frame_id is the location and direction of the reading

    float illuminance;          // Measurement of the Photometric Illuminance in Lux.

    float variance;              // 0 is interpreted as variance unknown
};

struct Image 
{
    std_msgs::Header header;     // Header timestamp should be acquisition time of image
                                 // Header frame_id should be optical frame of camera
                                 // origin of frame should be optical center of cameara
                                 // +x should point to the right in the image
                                 // +y should point down in the image
                                 // +z should point into to plane of the image
                                 // If the frame_id here and the frame_id of the CameraInfo
                                 // message associated with the image conflict
                                 // the behavior is undefined

    // image height, that is, number of rows
    uint32 height;       
    
    // image width, that is, number of columns
    uint32 width;            

    // The legal values for encoding are in file src/image_encodings.cpp
    // If you want to standardize a new string format, join
    // ros-users@lists.ros.org and send an email proposing a new encoding.
    std::string encoding;         // Encoding of pixels -- channel meaning, ordering, size
                                  // taken from the list of strings in include/sensor_msgs/image_encodings.hpp

    // is this data bigendian?
    uint32 is_bigendian;

    // Full row length in bytes
    uint32 step;

    // actual matrix data, size is (step * rows)
    std::vector<uint32> data;
};

struct Imu
{
    std_msgs::Header header;

    geometry_msgs::Quaternion orientation;
    std::vector<double> orientation_covariance;      // float64[9] orientation_covariance 
                                                     // Row major about x, y, z axes

    geometry_msgs::Vector3 angular_velocity;
    std::vector<double> angular_velocity_covariance;  // float64[9] angular_velocity_covariance  
                                                      // Row major about x, y, z axes

    geometry_msgs::Vector3 linear_acceleration;
    std::vector<double> linear_acceleration_covariance; // float64[9] linear_acceleration_covariance 
                                                        // Row major x, y z
};

struct LaserScan
{
    std_msgs::Header header;    // timestamp in the header is the acquisition time of
                                // the first ray in the scan.
                                //
                                // in frame frame_id, angles are measured around
                                // the positive Z axis (counterclockwise, if Z is up)
                                // with zero angle being forward along the x axis

    float angle_min;            // start angle of the scan [rad]
    float angle_max;            // end angle of the scan [rad]
    float angle_increment;      // angular distance between measurements [rad]

    float time_increment;       // time between measurements [seconds] - if your scanner
                                // is moving, this will be used in interpolating position
                                // of 3d points
    float scan_time;            // time between scans [seconds]

    float range_min;            // minimum range value [m]
    float range_max;            // maximum range value [m]

    std::vector<float>   ranges;    // range data [m]
                                    // (Note: values < range_min or > range_max should be discarded)
    std::vector<float> intensities; // intensity data [device-specific units].  If your
                                    // device does not provide intensities, please leave the array empty.
};

// This message holds the description of one point entry in the
// PointCloud2 message format.
// uint8 INT8    = 1
// uint8 UINT8   = 2
// uint8 INT16   = 3
// uint8 UINT16  = 4
// uint8 INT32   = 5
// uint8 UINT32  = 6
// uint8 FLOAT32 = 7
// uint8 FLOAT64 = 8
struct PointField
{
    // Common PointField names are x, y, z, intensity, rgb, rgba
    std::string name;       // Name of field
    uint32 offset;          // Offset from start of point struct
    uint32  datatype;       // Datatype enumeration, see above
    uint32 count;           // How many elements in the field
};

// This message holds a collection of 3d points, plus optional additional
// information about each point.
struct PointCloud
{
    // Time of sensor data acquisition, coordinate frame ID.
    std_msgs::Header header;

    // Array of 3d points. Each Point32 should be interpreted as a 3d point
    // in the frame given in the header.
    std::vector<geometry_msgs::Point32> points;

    // Each channel should have the same number of elements as points array,
    // and the data in each channel should correspond 1:1 with each point.
    // Channel names in common practice are listed in ChannelFloat32.msg.
    std::vector<ChannelFloat32> channels;
};

struct PointCloud2
{
    // Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
    std_msgs::Header header;

    // 2D structure of the point cloud. If the cloud is unordered, height is
    // 1 and width is the length of the point cloud.
    uint32 height;
    uint32 width = 3;

    // Describes the channels and their layout in the binary data blob.
    std::vector<PointField> fields;

    bool    is_bigendian; // Is this data bigendian?
    uint32  point_step;   // Length of a point in bytes
    uint32  row_step;     // Length of a row in bytes 
    std::vector<uint32> data; // Actual point data, size is (row_step*height)

    bool is_dense = 9;        // True if there are no invalid points
};

struct Range
{
    std_msgs::Header header;    // timestamp in the header is the time the ranger
                                // returned the distance reading

    // // Radiation type enums
    // // If you want a value added to this list, send an email to the ros-users list
    // uint8 ULTRASOUND=0
    // uint8 INFRARED=1
    uint32 radiation_type;      // the type of radiation used by the sensor(sound, IR, etc) [enum]

    float field_of_view;        // the size of the arc that the distance reading is
                                // valid for [rad]
                                // the object causing the range reading may have
                                // been anywhere within -field_of_view/2 and
                                // field_of_view/2 at the measured range.
                                // 0 angle corresponds to the x-axis of the sensor.

    float min_range;            // minimum range value [m]
    float max_range;            // maximum range value [m]
                                // Fixed distance rangers require min_range==max_range

    float range;                // range data [m]  
                                // (Note: values < range_min or > range_max should be discarded)
                                // Fixed distance rangers only output -Inf or +Inf.
                                // -Inf represents a detection within fixed distance.
                                // (Detection too close to the sensor to quantify)
                                // +Inf represents no detection within the fixed distance.
                                // (Object out of range)
};

struct RegionOfInterest
{
    uint32 x_offset;        // Leftmost pixel of the ROI
                            // (0 if the ROI includes the left edge of the image)
    uint32 y_offset;        // Topmost pixel of the ROI (0 if the ROI includes the top edge of the image)
    uint32 height;          // Height of ROI
    uint32 width ;          // Width of ROI

    // True if a distinct rectified ROI should be calculated from the "raw"
    // ROI in this message. Typically this should be False if the full image
    // is captured (ROI not used), and True if a subwindow is captured (ROI
    // used).
    bool do_rectify;
};

// ChannelFloat32.
proto::sensor_msgs::ChannelFloat32 ToProto(const ChannelFloat32& data);
ChannelFloat32 FromProto(const proto::sensor_msgs::ChannelFloat32& proto);

// CompressedImage
proto::sensor_msgs::CompressedImage ToProto(const CompressedImage& data);
CompressedImage FromProto(const proto::sensor_msgs::CompressedImage& proto);

// Illuminance
proto::sensor_msgs::Illuminance ToProto(const Illuminance& data);
Illuminance FromProto(const proto::sensor_msgs::Illuminance& proto);

// Image
proto::sensor_msgs::Image ToProto(const Image& data);
Image FromProto(const proto::sensor_msgs::Image& proto);

// Imu
proto::sensor_msgs::Imu ToProto(const Imu& data);
Imu FromProto(const proto::sensor_msgs::Imu& proto);

// LaserScan
proto::sensor_msgs::LaserScan ToProto(const LaserScan& data);
LaserScan FromProto(const proto::sensor_msgs::LaserScan& proto);

// PointField
proto::sensor_msgs::PointField ToProto(const PointField& data);
PointField FromProto(const proto::sensor_msgs::PointField& proto);

// PointCloud
proto::sensor_msgs::PointCloud ToProto(const PointCloud& data);
PointCloud FromProto(const proto::sensor_msgs::PointCloud& proto);

// PointCloud2
proto::sensor_msgs::PointCloud2 ToProto(const PointCloud2& data);
PointCloud2 FromProto(const proto::sensor_msgs::PointCloud2& proto);

// Range
proto::sensor_msgs::Range ToProto(const Range& data);
Range FromProto(const proto::sensor_msgs::Range& proto);

// RegionOfInterest
proto::sensor_msgs::RegionOfInterest ToProto(const RegionOfInterest& data);
RegionOfInterest FromProto(const proto::sensor_msgs::RegionOfInterest& proto);

}  // namespace sensor_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_SENSOR_MSGS_HPP_
