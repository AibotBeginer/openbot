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
#include <cstddef>


namespace builtin_interfaces {
namespace msg {


// Time indicates a specific point in time, relative to a clock's 0 point.
struct Time
{
    // The seconds component, valid over all int32 values.
    int32_t sec;

    // The nanoseconds component, valid in the range [0, 10e9).
    uint32_t nanosec;
};


// Duration defines a period between two time points. It is comprised of a
// seconds component and a nanoseconds component.
struct Duration
{
    // Seconds component, range is valid over any possible int32 value.
    int32_t sec;

    // Nanoseconds component in the range of [0, 10e9).
    uint32_t nanosec;
};

}  // namespace msg
}  // namespace builtin_interfaces