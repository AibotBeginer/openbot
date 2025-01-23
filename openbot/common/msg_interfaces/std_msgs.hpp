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

#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <cstddef>

#include "openbot/common/msg_interfaces/builtin_interfaces.hpp"

namespace std_msgs {
namespace msg {

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct Bool
{
    bool data;
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct Byte
{
    std::byte data;
};

struct ColorRGBA
{
    float r;
    float g;
    float b;
    float a;
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct MultiArrayDimension
{
    std::string label;   // label of given dimension
    uint32_t size;       // size of given dimension (in type units)
    uint32_t stride;     // stride of given dimension
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.

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
struct MultiArrayLayout
{
    std::vector<MultiArrayDimension> dim; // Array of dimension properties
    uint32_t data_offset;                 // padding bytes at front of data
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.

// Please look at the MultiArrayLayout message definition for
// documentation on all multiarrays.
struct ByteMultiArray
{
    MultiArrayLayout  layout;       // specification of data layout
    std::vector<std::byte> data ;        // array of data
};

// Standard metadata for higher-level stamped data types.
// This is generally used to communicate timestamped data
// in a particular coordinate frame.
struct Header
{
    // Two-integer timestamp that is expressed as seconds and nanoseconds.
    builtin_interfaces::msg::Time stamp;

    // Transform frame with which this data is associated.
    std::string frame_id;
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct String
{
    std::string data;
};

}  // namespace msg
}  // namespace std_msgs