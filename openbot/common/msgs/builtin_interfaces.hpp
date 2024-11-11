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

#ifndef OPENBOT_COMMON_MSGS_BUILTIN_INTERFACES_HPP_
#define OPENBOT_COMMON_MSGS_BUILTIN_INTERFACES_HPP_

// std_msgs
#include "openbot/common/proto/builtin_interfaces/time.pb.h"
#include "openbot/common/port.hpp"

#include <iostream>

namespace openbot {
namespace common {
namespace builtin_interfaces {

struct Time 
{
    // The seconds component, valid over all int32 values.
    uint32 sec;

    // The nanoseconds component, valid in the range [0, 10e9).
    uint32 nanosec;

    Time& operator=(const uint64_t timestamp) {
        // Convert the timestamp to seconds and nanoseconds
        sec = static_cast<uint32_t>(timestamp / 1000000000);
        nanosec = static_cast<uint32_t>(timestamp % 1000000000);
        return *this;
    }

    Time() : sec(0), nanosec(0) {}

    Time(int s) : sec(s), nanosec(0) {}

    Time(int s, int n) : sec(s), nanosec(n) {}

    static Time now() 
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time.time_since_epoch());
        Time result;
        result.sec = duration_since_epoch.count() / 1000000000;
        result.nanosec = duration_since_epoch.count() % 1000000000;
        return result;
    }

    bool operator==(const Time& other) const 
    {
        return sec == other.sec && nanosec == other.nanosec;
    }

    bool operator==(const int rhs) const 
    {
        return sec == rhs;
    }

    bool operator!=(const Time& other) const 
    {
        return sec!= other.sec || nanosec!= other.nanosec;
    }

    bool operator<(const Time& other) const 
    {
        if (sec < other.sec) {
            return true;
        } else if (sec > other.sec) {
            return false;
        } else {
            return nanosec < other.nanosec;
        }
    }

    bool operator<=(const Time& other) const 
    {
        if (sec < other.sec) {
            return true;
        } else if (sec > other.sec) {
            return false;
        } else {
            return nanosec <= other.nanosec;
        }
    }

    bool operator>(const Time& rhs) const 
    {
        if (sec > rhs.sec) {
            return true;
        } else if (sec < rhs.sec) {
            return false;
        } else {
            return nanosec > rhs.nanosec;
        }
    }

    bool operator>=(const Time& rhs) const {
        if (sec > rhs.sec) {
            return true;
        } else if (sec < rhs.sec) {
            return false;
        } else {
            return nanosec >= rhs.nanosec;
        }
    }

    Time operator-(const Time& rhs) const {
        Time result;
        result.sec = sec - rhs.sec;
        if (result.sec < 0) {
            result.sec += 60;
            result.nanosec -= 1000000000;
        }
        result.nanosec -= rhs.nanosec;
        if (result.nanosec < 0) {
            result.nanosec += 1000000000;
            result.sec--;
        }
        return result;
    }

    Time operator+(const Time& rhs) const {
        Time result;
        result.sec = sec + rhs.sec;
        result.nanosec = nanosec + rhs.nanosec;
        if (result.nanosec >= 1000000000) {
            result.sec++;
            result.nanosec -= 1000000000;
        }
        return result;
    }

    Time operator+(const uint64_t rhs) const 
    {
        uint64_t total_nanos = sec * 1000000000 + nanosec + rhs * 1000000000;
        uint32_t new_sec = total_nanos / 1000000000;
        uint32_t new_nanosec = total_nanos % 1000000000;
        return {new_sec, new_nanosec};
    }
    
};

struct Duration
{

};

// Overload the << operator for the Time struct
std::ostream& operator<<(std::ostream& os, const Time& time);

// Converts 'Time' to a openbot::common::proto::std_msgs::Header.
openbot::common::proto::builtin_interfaces::Time ToProto(const Time& data);

// Converts 'proto' to openbot::common::proto::Header.
Time FromProto(const openbot::common::proto::builtin_interfaces::Time& proto);

}  // namespace builtin_interfaces
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_MSGS_BUILTIN_INTERFACES_HPP_
