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

#ifndef OPENBOT_COMMON_MSGS_STD_MSGS_HPP_
#define OPENBOT_COMMON_MSGS_STD_MSGS_HPP_

#include <string>

#include "openbot/common/proto/std_msgs/header.pb.h"
#include "openbot/common/msgs/builtin_interfaces.hpp"

namespace openbot {
namespace common {
namespace std_msgs {

struct Header
{
    builtin_interfaces::Time stamp;

    // Transform frame with which this data is associated.
    std::string frame_id;
};

struct MultiArrayDimension
{
    std::string label;   // label of given dimension
    uint32 size;         // size of given dimension (in type units)
    uint32 stride;       // stride of given dimension
};

struct MultiArrayLayout
{
    // The multiarray declares a generic multi-dimensional array of a
    // particular data type.  Dimensions are ordered from outer most
    // to inner most.
    //
    // Accessors should ALWAYS be written in terms of dimension stride
    // and specified outer-most dimension first.
    //
    // multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    //
    // A standard, 3-channel 640x480 image with interleaved color channels
    // would be specified as:
    //
    // dim[0].label  = "height"
    // dim[0].size   = 480
    // dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    // dim[1].label  = "width"
    // dim[1].size   = 640
    // dim[1].stride = 3*640 = 1920
    // dim[2].label  = "channel"
    // dim[2].size   = 3
    // dim[2].stride = 3
    //
    // multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

    std::vector<MultiArrayDimension> dim;       // Array of dimension properties
    uint32 data_offset;                         // padding bytes at front of data
};

struct Float32MultiArray
{
    // Please look at the MultiArrayLayout message definition for
    // documentation on all multiarrays.

    MultiArrayLayout layout;        // specification of data layout
    std::vector<float> data;        // array of data
};

// Converts 'Header' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::std_msgs::Header ToProto(const Header& data);

// Converts 'proto' to openbot::common::proto::Header.
Header FromProto(const openbot::common::proto::std_msgs::Header& proto);

}  // namespace std_msgs
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_STD_MSGS_HPP_
