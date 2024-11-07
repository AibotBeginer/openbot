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

#include "openbot/common/motion_model/stationary_motion_model.hpp"
#include "openbot/common/state_vector/common_states.hpp"

#include <gtest/gtest.h>

namespace openbot {
namespace common { 
namespace motion_model {

using openbot::common::state_vector::variable::X;
using openbot::common::state_vector::variable::Y;
using openbot::common::state_vector::FloatState;

/// @test Test that prediction on independent x, y works as expected.
TEST(StationaryMotionModelTest, Predict) {
  StationaryMotionModel<FloatState<X, Y>> model{};
  FloatState<X, Y> state{};
  state.at<X>() = 42.0F;
  state.at<Y>() = 23.0F;
  EXPECT_FLOAT_EQ(model.predict(state, std::chrono::milliseconds{42}).at<X>(), state.at<X>());
  EXPECT_FLOAT_EQ(model.predict(state, std::chrono::milliseconds{42}).at<Y>(), state.at<Y>());
}

/// @test Test that prediction on independent x, y works as expected.
TEST(StationaryMotionModelTest, Jacobian) {
  StationaryMotionModel<FloatState<X, Y>> model{};
  FloatState<X, Y> state{};
  EXPECT_TRUE(
    model.jacobian(state, std::chrono::milliseconds{42}).isApprox(Eigen::Matrix2f::Identity()));
}

}  // namespace motion_model
}  // namespace common 
}  // namespace openbot