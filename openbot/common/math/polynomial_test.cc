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

#include "openbot/common/math/polynomial.hpp"

#include <gtest/gtest.h>

namespace openbot {
namespace common {
namespace math {

namespace {

#define CHECK_EQUAL_RESULT(find_func1, coeffs1, find_func2, coeffs2) \
  {                                                                  \
    Eigen::VectorXd real1;                                           \
    Eigen::VectorXd imag1;                                           \
    const bool success1 = find_func1(coeffs1, &real1, &imag1);       \
    Eigen::VectorXd real2;                                           \
    Eigen::VectorXd imag2;                                           \
    const bool success2 = find_func2(coeffs2, &real2, &imag2);       \
    EXPECT_EQ(success1, success2);                                   \
    if (success1) {                                                  \
      EXPECT_EQ(real1, real2);                                       \
      EXPECT_EQ(imag1, imag2);                                       \
    }                                                                \
  }

TEST(EvaluatePolynomial, Nominal) {
  EXPECT_EQ(EvaluatePolynomial(
                (Eigen::VectorXd(5) << 1, -3, 3, -5, 10).finished(), 1),
            1 - 3 + 3 - 5 + 10);
  EXPECT_NEAR(
      EvaluatePolynomial((Eigen::VectorXd(4) << 1, -3, 3, -5).finished(), 2.0),
      1 * 2 * 2 * 2 - 3 * 2 * 2 + 3 * 2 - 5,
      1e-6);
}

TEST(FindLinearPolynomialRoots, Nominal) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  EXPECT_TRUE(FindLinearPolynomialRoots(Eigen::Vector2d(3, -2), &real, &imag));
  EXPECT_EQ(real(0), 2.0 / 3.0);
  EXPECT_EQ(imag(0), 0);
  EXPECT_NEAR(EvaluatePolynomial(Eigen::Vector2d(3, -2),
                                 std::complex<double>(real(0), imag(0)))
                  .real(),
              0.0,
              1e-6);
  EXPECT_NEAR(EvaluatePolynomial(Eigen::Vector2d(3, -2),
                                 std::complex<double>(real(0), imag(0)))
                  .imag(),
              0.0,
              1e-6);

  EXPECT_FALSE(FindLinearPolynomialRoots(Eigen::Vector2d(0, 1), &real, &imag));
}

TEST(FindQuadraticPolynomialRootsReal, Nominal) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  Eigen::Vector3d coeffs(3, -2, -4);
  EXPECT_TRUE(FindQuadraticPolynomialRoots(coeffs, &real, &imag));
  EXPECT_TRUE(real.isApprox(Eigen::Vector2d(-0.868517092, 1.535183758), 1e-6));
  EXPECT_EQ(imag, Eigen::Vector2d(0, 0));
  EXPECT_NEAR(
      EvaluatePolynomial(coeffs, std::complex<double>(real(0), imag(0))).real(),
      0.0,
      1e-6);
  EXPECT_NEAR(
      EvaluatePolynomial(coeffs, std::complex<double>(real(1), imag(1))).imag(),
      0.0,
      1e-6);
}

TEST(FindQuadraticPolynomialRootsComplex, Nominal) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  const Eigen::Vector3d coeffs(
      0.276025076998578, 0.679702676853675, 0.655098003973841);
  EXPECT_TRUE(FindQuadraticPolynomialRoots(coeffs, &real, &imag));
  EXPECT_TRUE(real.isApprox(
      Eigen::Vector2d(-1.231233560813707, -1.231233560813707), 1e-6));
  EXPECT_TRUE(imag.isApprox(
      Eigen::Vector2d(0.925954520440279, -0.925954520440279), 1e-6));
  EXPECT_NEAR(
      EvaluatePolynomial(coeffs, std::complex<double>(real(0), imag(0))).real(),
      0.0,
      1e-6);
  EXPECT_NEAR(
      EvaluatePolynomial(coeffs, std::complex<double>(real(1), imag(1))).imag(),
      0.0,
      1e-6);
}

TEST(FindCubicPolynomialRoots, SingleRoot) {
  const Eigen::Vector4d coeffs(
      1, 0.276025076998578, 0.679702676853675, 0.655098003973841);
  Eigen::Vector3d real;
  EXPECT_EQ(FindCubicPolynomialRoots(coeffs(1), coeffs(2), coeffs(3), &real),
            1);
  EXPECT_NEAR(real(0), -0.68359403879256575, 1e-6);
  EXPECT_NEAR(
      EvaluatePolynomial(coeffs, std::complex<double>(real(0), 0)).real(),
      0.0,
      1e-6);
}

TEST(FindCubicPolynomialRoots, MultiRoot) {
  const Eigen::Vector4d coeffs(1, -3, -3, 5);
  Eigen::Vector3d real;
  EXPECT_EQ(FindCubicPolynomialRoots(coeffs(1), coeffs(2), coeffs(3), &real),
            3);
  std::sort(real.data(), real.data() + real.size());
  EXPECT_TRUE(real.isApprox(
      Eigen::Vector3d(-1.4494897427831781, 1, 3.4494897427831783), 1e-6));
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(
        EvaluatePolynomial(coeffs, std::complex<double>(real(i), 0)).real(),
        0.0,
        1e-6);
  }
  Eigen::VectorXd real_durand_kerner;
  EXPECT_TRUE(
      FindPolynomialRootsDurandKerner(coeffs, &real_durand_kerner, nullptr));
  std::sort(real_durand_kerner.data(),
            real_durand_kerner.data() + real_durand_kerner.size());
  EXPECT_TRUE(real.isApprox(real_durand_kerner, 1e-4));
}

TEST(FindPolynomialRootsDurandKerner, Nominal) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  Eigen::VectorXd coeffs(5);
  coeffs << 10, -5, 3, -3, 1;
  EXPECT_TRUE(FindPolynomialRootsDurandKerner(coeffs, &real, &imag));
  // Reference values generated with OpenCV/Matlab.
  Eigen::VectorXd ref_real(4);
  ref_real << -0.201826, -0.201826, 0.451826, 0.451826;
  EXPECT_TRUE(real.isApprox(ref_real, 1e-6));
  Eigen::VectorXd ref_imag(4);
  ref_imag << -0.627696, 0.627696, 0.160867, -0.160867;
  EXPECT_TRUE(imag.isApprox(ref_imag, 1e-6));
}

TEST(FindPolynomialRootsDurandKernerLinearQuadratic, Nominal) {
  CHECK_EQUAL_RESULT(FindPolynomialRootsDurandKerner,
                     Eigen::Vector2d(1, 2),
                     FindLinearPolynomialRoots,
                     Eigen::Vector2d(1, 2));
  CHECK_EQUAL_RESULT(FindPolynomialRootsDurandKerner,
                     (Eigen::VectorXd(4) << 0, 0, 1, 2).finished(),
                     FindLinearPolynomialRoots,
                     Eigen::Vector2d(1, 2));
  CHECK_EQUAL_RESULT(FindPolynomialRootsDurandKerner,
                     Eigen::Vector3d(1, 2, 3),
                     FindQuadraticPolynomialRoots,
                     Eigen::Vector3d(1, 2, 3));
  CHECK_EQUAL_RESULT(FindPolynomialRootsDurandKerner,
                     (Eigen::VectorXd(5) << 0, 0, 1, 2, 3).finished(),
                     FindQuadraticPolynomialRoots,
                     Eigen::Vector3d(1, 2, 3));
}

TEST(FindPolynomialRootsCompanionMatrix, Nominal) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  Eigen::VectorXd coeffs(5);
  coeffs << 10, -5, 3, -3, 1;
  EXPECT_TRUE(FindPolynomialRootsCompanionMatrix(coeffs, &real, &imag));
  // Reference values generated with OpenCV/Matlab.
  Eigen::VectorXd ref_real(4);
  ref_real << -0.201826, -0.201826, 0.451826, 0.451826;
  EXPECT_TRUE(real.isApprox(ref_real, 1e-6));
  Eigen::VectorXd ref_imag(4);
  ref_imag << 0.627696, -0.627696, 0.160867, -0.160867;
  EXPECT_TRUE(imag.isApprox(ref_imag, 1e-6));
}

TEST(FindPolynomialRootsCompanionMatrixLinearQuadratic, Nominal) {
  CHECK_EQUAL_RESULT(FindPolynomialRootsCompanionMatrix,
                     Eigen::Vector2d(1, 2),
                     FindLinearPolynomialRoots,
                     Eigen::Vector2d(1, 2));
  CHECK_EQUAL_RESULT(FindPolynomialRootsCompanionMatrix,
                     (Eigen::VectorXd(4) << 0, 0, 1, 2).finished(),
                     FindLinearPolynomialRoots,
                     Eigen::Vector2d(1, 2));
  CHECK_EQUAL_RESULT(FindPolynomialRootsCompanionMatrix,
                     Eigen::Vector3d(1, 2, 3),
                     FindQuadraticPolynomialRoots,
                     Eigen::Vector3d(1, 2, 3));
  CHECK_EQUAL_RESULT(FindPolynomialRootsCompanionMatrix,
                     (Eigen::VectorXd(5) << 0, 0, 1, 2, 3).finished(),
                     FindQuadraticPolynomialRoots,
                     Eigen::Vector3d(1, 2, 3));
}

TEST(FindPolynomialRootsCompanionMatrixZeroSolution, Nominal) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  Eigen::VectorXd coeffs(5);
  coeffs << 10, -5, 3, -3, 0;
  EXPECT_TRUE(FindPolynomialRootsCompanionMatrix(coeffs, &real, &imag));
  // Reference values generated with Matlab.
  Eigen::VectorXd ref_real(4);
  ref_real << 0.692438, -0.0962191, -0.0962191, 0;
  EXPECT_TRUE(real.isApprox(ref_real, 1e-6));
  Eigen::VectorXd ref_imag(4);
  ref_imag << 0, 0.651148, -0.651148, 0;
  EXPECT_TRUE(imag.isApprox(ref_imag, 1e-6));
}

}  // namespace
}  // namespace math
}  // namespace common
}  // namespace openbot

