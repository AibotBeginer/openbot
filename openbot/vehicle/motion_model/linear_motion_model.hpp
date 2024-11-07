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

#ifndef OPENBOT_VEHICLE_MOTION_MODEL_LINEAR_MOTION_MODEL_HPP_
#define OPENBOT_VEHICLE_MOTION_MODEL_LINEAR_MOTION_MODEL_HPP_

#include "openbot/vehicle/motion_model/motion_model_interface.hpp"

#include "openbot/common/state_vector/common_states.hpp"
#include "openbot/common/state_vector/generic_state.hpp"

namespace openbot {
namespace vehicle { 
namespace motion_model {

///
/// @brief      A generic linear motion model class.
///
/// @details    This class is designed to handle all the linear motion models, i.e., those that only
///             have independent variables in them. In this case, the Jacobian is just a transition
///             matrix.
///
/// @tparam     StateT  State type.
///
template<typename StateT>
class LinearMotionModel : public MotionModelInterface<LinearMotionModel<StateT>>
{
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<LinearMotionModel<StateT>>;

  ///
  /// @brief      A crtp-called function that predicts the state forward.
  ///
  /// @param[in]  state  The current state vector
  /// @param[in]  dt     Time difference
  ///
  /// @return     New state after prediction.
  ///
  inline State crtp_predict(const State & state, const std::chrono::nanoseconds& dt) const
  {
    return State{crtp_jacobian(state, dt) * state.vector()};
  }

  ///
  /// @brief      A crtp-called function that computes a Jacobian.
  ///
  /// @note       The default implementation assumes that all variables have position, velocity and
  ///             acceleration entries. If a custom state that does not follow this convention is to
  ///             be used a specialization of this function must be added.
  ///
  /// @return     A matrix that represents the Jacobian.
  ///
  typename State::Matrix crtp_jacobian(const State &, const std::chrono::nanoseconds & dt) const;
};

}  // namespace motion_model
}  // namespace vehicle 
}  // namespace openbot

#endif  // OPENBOT_VEHICLE_MOTION_MODEL_LINEAR_MOTION_MODEL_HPP_
