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


#ifndef OPENBOT_COMMON_TF2_TRANSFORM_STORAGE_HPP_
#define OPENBOT_COMMON_TF2_TRANSFORM_STORAGE_HPP_

#include "openbot/common/tf2/LinearMath/Vector3.hpp"
#include "openbot/common/tf2/LinearMath/Quaternion.hpp"
#include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/tf2/time.hpp"

namespace openbot {
namespace common {
namespace tf2 {

typedef uint32_t CompactFrameID;

/** \brief Storage for transforms and their parent */
class TransformStorage
{
public:
  TransformStorage();
  TransformStorage(const geometry_msgs::TransformStamped& data, CompactFrameID frame_id, CompactFrameID child_frame_id);

  TransformStorage(const TransformStorage& rhs)
  {
    *this = rhs;
  }

  TransformStorage& operator=(const TransformStorage& rhs)
  {
#if 01
    rotation_ = rhs.rotation_;
    translation_ = rhs.translation_;
    stamp_ = rhs.stamp_;
    frame_id_ = rhs.frame_id_;
    child_frame_id_ = rhs.child_frame_id_;
#endif
    return *this;
  }

  tf2::Quaternion rotation_;
  tf2::Vector3 translation_;
  Time stamp_;
  CompactFrameID frame_id_;
  CompactFrameID child_frame_id_;
};

}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif // OPENBOT_COMMON_TF2_TRANSFORM_STORAGE_HPP_

