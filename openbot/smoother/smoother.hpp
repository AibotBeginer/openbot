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

#ifndef OPENBOT_SMOOTHER_SMOOTHER_HPP_
#define OPENBOT_SMOOTHER_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "openbot/common/macros.hpp"
#include "openbot/common/utils/time.hpp"
#include "openbot/common/proto/nav_msgs/path.pb.h"

namespace openbot {
namespace smoother { 

class Smoother
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(Smoother);

    /**
     * @brief Virtual destructor
     */
    virtual ~Smoother() {}


    virtual void Configure() = 0;

    /**
     * @brief Method to cleanup resources.
     */
    virtual void Cleanup() = 0;

    /**
     * @brief Method to activate smoother and any threads involved in execution.
     */
    virtual void Activate() = 0;

    /**
     * @brief Method to deactivate smoother and any threads involved in execution.
     */
    virtual void Deactivate() = 0;

    /**
     * @brief Method to smooth given path
     *
     * @param path In-out path to be smoothed
     * @param max_time Maximum duration smoothing should take
     * @return If smoothing was completed (true) or interrupted by time limit (false)
     */
    virtual bool Smooth(
        common::proto::nav_msgs::Path& path,
        const common::Duration & max_time) = 0;
};

}  // namespace smoother 
}  // namespace openbot

#endif  // OPENBOT_SMOOTHER_SMOOTHER_HPP_
