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


#ifndef OPENBOT_COMMON_TF2_TRANSFORM_DATATYPES_HPP_
#define OPENBOT_COMMON_TF2_TRANSFORM_DATATYPES_HPP_

#include <string>

#include "openbot/common/tf2/time.hpp"

namespace openbot {
namespace common {
namespace tf2 {

/** \brief The data type which will be cross compatable with geometry_msgs
 * This is the tf2 datatype equivilant of a MessageStamped */
template <typename T>
class Stamped : public T{
 public:
  Time stamp_; ///< The timestamp associated with this data
  std::string frame_id_; ///< The frame_id associated this data

  /** Default constructor */
  Stamped() :frame_id_ ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){}; //Default constructor used only for preallocation

  /** Full constructor */
  Stamped(const T& input, const Time& timestamp, const std::string & frame_id) :
    T (input), stamp_ ( timestamp ), frame_id_ (frame_id){ } ;
  
  /** Copy Constructor */
  Stamped(const Stamped<T>& s):
    T (s),
    stamp_(s.stamp_),
    frame_id_(s.frame_id_) {}
  
  /** Set the data element */
  void setData(const T& input){*static_cast<T*>(this) = input;};
};

/** \brief Comparison Operator for Stamped datatypes */
template <typename T> 
bool operator==(const Stamped<T> &a, const Stamped<T> &b) {
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ && static_cast<const T&>(a) == static_cast<const T&>(b);
};


}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif // OPENBOT_COMMON_TF2_TRANSFORM_DATATYPES_HPP_
