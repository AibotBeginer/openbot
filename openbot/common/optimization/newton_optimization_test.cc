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

#include "openbot/common/optimization/newton_optimization_test.hpp"
#include "openbot/common/optimization/line_search/fixed_line_search.hpp"
#include "openbot/common/helper_functions/types.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace openbot {
namespace common {
namespace optimization {

using openbot::common::types::float64_t;

using NewtonPolynomialTestParam1D = Polynomial1dParam<OptimizationOptions, FixedLineSearch>;
class NewtonOptimizationParamTest
  : public ::testing::TestWithParam<NewtonPolynomialTestParam1D> {};

TEST_P(NewtonOptimizationParamTest, NewtonOptimizationValidation) {
  auto problem = GetParam().problem;
  auto objective = problem.get_objective();
  const auto x0 = GetParam().x0;
  const auto options = GetParam().options;
  const auto line_search = GetParam().line_searcher;
  const auto expected_termination = GetParam().expected_termination;
  const auto solution = objective.solution;
  const auto is_convex = objective.convex;

  NewtonsMethodOptimizer<FixedLineSearch> optimizer{line_search, options};

  decltype(problem)::DomainValue x_out;
  const auto summary = optimizer.solve(problem, x0, x_out);
  EXPECT_EQ(summary.termination_type(), expected_termination);
  if (is_convex && (expected_termination == TerminationType::CONVERGENCE)) {
    EXPECT_FLOAT_EQ(x_out(0, 0), solution);
    EXPECT_LE(summary.estimated_distance_to_optimum(), line_search.get_step_max());
  }
}


TEST(TestFixedLineSearch, FixedLineSearchValidation) {
  // set up varaibles
  constexpr auto step = 0.01F;
  Polynomial1DOptimizationProblem dummy_optimization_problem{-2.0, 16, 10.0};


  // test derived class
  FixedLineSearch fls;
  EXPECT_FLOAT_EQ(fls.get_step_max(), std::numeric_limits<float_t>::min());
  fls.set_step_max(step);
  // TODO(yunus.caliskan): enable in #308
  //  EXPECT_FLOAT_EQ(fls.compute_next_step(dummy_optimization_problem), step);
}

}  // namespace optimization
}  // namespace common
}  // namespace openbot