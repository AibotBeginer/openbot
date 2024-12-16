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


#ifndef OPENBOT_COMMON_COVARIANCE_INSERTION__OUTPUT_TYPE_TRAIT_HPP_
#define OPENBOT_COMMON_COVARIANCE_INSERTION__OUTPUT_TYPE_TRAIT_HPP_

// #include "openbot/common/msgs/msgs.hpp"
#include "openbot/common/proto/geometry_msgs.pb.h"

namespace openbot {
namespace common {

template<typename InputT>
struct output
{
  using type = InputT;
};

template<typename InputT, typename Enable = void>
struct needs_conversion : public std::true_type {};

template<typename InputT>
struct needs_conversion<InputT, std::enable_if_t<
    std::is_same<typename output<InputT>::type, InputT>::value>>: public std::false_type {};

// Specializations.
template<>
struct output<geometry_msgs::Pose>
{
  using type = geometry_msgs::PoseWithCovariance;
};

template<>
struct output<geometry_msgs::PoseStamped>
{
  using type = geometry_msgs::PoseWithCovarianceStamped;
};

template<>
struct output<geometry_msgs::Twist>
{
  using type = geometry_msgs::TwistWithCovariance;
};

template<>
struct output<geometry_msgs::TwistStamped>
{
  using type = geometry_msgs::TwistWithCovarianceStamped;
};


}  // namespace common
}  // namespace openbot

#endif  // COVARIANCE_INSERTION__OUTPUT_TYPE_TRAIT_HPP_
