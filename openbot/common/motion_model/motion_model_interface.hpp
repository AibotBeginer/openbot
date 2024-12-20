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

#ifndef OPENBOT_COMMON_MOTION_MODEL_MOTION_MODEL_INTERFACE_HPP_
#define OPENBOT_COMMON_MOTION_MODEL_MOTION_MODEL_INTERFACE_HPP_

#include "openbot/common/helper_functions/crtp.hpp"
#include "openbot/common/state_vector/generic_state.hpp"

#include <chrono>

namespace openbot {
namespace common { 
namespace motion_model {

///
/// @brief      A CRTP interface for any motion model.
///
/// @tparam     Derived  Motion model implementation class.
///
template<typename Derived>
class MotionModelInterface : public common::helper_functions::crtp<Derived>
{
public:
  ///
  /// @brief      Get the next predicted state.
  ///
  /// @param[in]  state   The initial state
  /// @param[in]  dt      Length of prediction into the future
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     Predicted state after the time has passed.
  ///
  template<typename StateT>
  inline auto predict(const StateT & state, const std::chrono::nanoseconds & dt) const
  {
    static_assert(
      common::state_vector::is_state<StateT>::value,
      "\n\nStateT must be a GenericState\n\n");
    return this->impl().crtp_predict(state, dt);
  }
  ///
  /// @brief      Get the Jacobian of this motion model.
  ///
  /// @param[in]  state   The current state.
  /// @param[in]  dt      Time span.
  ///
  /// @tparam     StateT  Type of the state.
  ///
  /// @return     A Jacobian of the motion model. In the linear case - a transition matrix.
  ///
  template<typename StateT>
  inline auto jacobian(const StateT & state, const std::chrono::nanoseconds & dt) const
  {
    static_assert(
      common::state_vector::is_state<StateT>::value,
      "\n\nStateT must be a GenericState\n\n");
    return this->impl().crtp_jacobian(state, dt);
  }
};

}  // namespace motion_model
}  // namespace common 
}  // namespace openbot

#endif  // OPENBOT_COMMON_MOTION_MODEL_MOTION_MODEL_INTERFACE_HPP_
