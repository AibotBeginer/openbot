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

#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/optional.h"
#include "openbot/drivers/sensor/common/data.hpp"

namespace openbot {
namespace drivers {
namespace sensor { 

class CollatorInterface 
{
public:
    using Callback = std::function<void(const std::string&, std::unique_ptr<Data>)>;

    CollatorInterface() {}
    virtual ~CollatorInterface() {}
    CollatorInterface(const CollatorInterface&) = delete;
    CollatorInterface& operator=(const CollatorInterface&) = delete;

    // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
    // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
    // in time order.
    virtual void AddSensorData(std::unique_ptr<Data> data) = 0;

    // Dispatches all queued sensor packets. May only be called once.
    // AddSensorData may not be called after Flush.
    virtual void Flush() = 0;
};

}  // namespace sensor
}  // namespace drivers
}  // namespace openbot