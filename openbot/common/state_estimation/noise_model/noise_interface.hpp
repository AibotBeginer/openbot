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


#ifndef OPENBOT_COMMON_STATE_ESTIMATION_NOISE_MODEL_NOISE_INTERFACE_HPP_
#define OPENBOT_COMMON_STATE_ESTIMATION_NOISE_MODEL_NOISE_INTERFACE_HPP_

#include <chrono>

namespace openbot {
namespace common {
namespace state_estimation {

///
/// @brief      A CRTP interface for implementing noise models used for providing motion model noise
///             covariance.
///
/// @tparam     Derived  A derived class that holds the actual implementation.
///
template<typename Derived>
class NoiseInterface
{
public:
  ///
  /// @brief      Get a covariance matrix for this noise model.
  ///
  /// @param[in]  dt    Time difference.
  ///
  /// @return     A covariance matrix for the noise gain during the dt time difference.
  ///
  inline auto covariance(const std::chrono::nanoseconds & dt) const
  {
    return static_cast<const Derived &>(*this).crtp_covariance(dt);
  }
};

}  // namespace state_estimation
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_STATE_ESTIMATION_NOISE_MODEL_NOISE_INTERFACE_HPP_
