/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testKnownLandmarkSquaredRangeFactor.cpp
 * @brief Tests for KnownLandmarkSquaredRangeFactor.
 * @author Oluwatodimu Afolayan
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/KnownLandmarkSquaredRangeFactor.h>

#include <Eigen/Core>
#include <memory>
#include <stdexcept>

using namespace gtsam;

/* ************************************************************************* */
namespace point2_fixture {

using Point = Eigen::Matrix<double, 2, 1>;
using Factor = KnownLandmarkSquaredRangeFactor<2>;

const Key kKey = 3;

// Verifies the 2D residual, analytic Jacobian, and zero-residual case.
TEST(KnownLandmarkSquaredRangeFactor, Point2) {
  const Point wX(1.0, 2.0);
  const Point wL(4.0, 6.0);
  const double measuredRange = 4.0;
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, measuredRange, model);

  Matrix H;
  const Vector actual = factor.evaluateError(wX, H);
  const Vector expected = Vector1(9.0);

  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(1, H.rows());
  EXPECT_LONGS_EQUAL(2, H.cols());

  Matrix expectedH(1, 2);
  expectedH << -6.0, -8.0;
  EXPECT(assert_equal(expectedH, H));

  const Factor exactFactor(kKey, wL, 5.0, model);
  EXPECT(assert_equal(Vector1::Zero(), exactFactor.evaluateError(wX)));

  Values values;
  values.insert(kKey, wX);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that scalar information weighting produces the expected factor cost.
TEST(KnownLandmarkSquaredRangeFactor, ScalarInfo) {
  const Point wX(1.0, 2.0);
  const Point wL(4.0, 6.0);

  Matrix information(1, 1);
  information << 4.0;
  const auto model = noiseModel::Gaussian::Information(information);
  const Factor factor(kKey, wL, 4.0, model);

  Values values;
  values.insert(kKey, wX);

  const Vector error = factor.evaluateError(wX);
  const double expected = 0.5 * error.dot(information * error);

  EXPECT_DOUBLES_EQUAL(162.0, expected, 1e-12);
  EXPECT_DOUBLES_EQUAL(expected, factor.error(values), 1e-9);
}

// Verifies that dynamically sized 2D landmarks are accepted after validation.
TEST(KnownLandmarkSquaredRangeFactor, AcceptsDynamicLandmark2D) {
  Vector wL(2);
  wL << 4.0, 6.0;
  const Point wX(1.0, 2.0);
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, 4.0, model);
  EXPECT(assert_equal(Vector1(9.0), factor.evaluateError(wX)));
}

// Verifies that short and long 2D landmark vectors throw before Eigen
// conversion.
TEST(KnownLandmarkSquaredRangeFactor, RejectsWrongLandmarkDimension2D) {
  const auto model = noiseModel::Unit::Create(1);
  const Vector tooShort = Vector::Zero(1);
  const Vector tooLong = Vector::Zero(3);

  CHECK_EXCEPTION(Factor(kKey, tooShort, 4.0, model), std::invalid_argument);
  CHECK_EXCEPTION(Factor(kKey, tooLong, 4.0, model), std::invalid_argument);
}

// Verifies that negative range measurements are rejected.
TEST(KnownLandmarkSquaredRangeFactor, RejectsNegativeRange) {
  const Point wL(4.0, 6.0);
  const auto model = noiseModel::Unit::Create(1);

  CHECK_EXCEPTION(Factor(kKey, wL, -4.0, model), std::invalid_argument);
}

}  // namespace point2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace point3_fixture {

using Point = Eigen::Matrix<double, 3, 1>;
using Factor = KnownLandmarkSquaredRangeFactor<3>;

const Key kKey = 4;

// Verifies the 3D residual, analytic Jacobian, and zero-residual case.
TEST(KnownLandmarkSquaredRangeFactor, Point3) {
  const Point wX(1.0, 2.0, 3.0);
  const Point wL(3.0, 5.0, 9.0);
  const double measuredRange = 5.0;
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, measuredRange, model);

  Matrix H;
  const Vector actual = factor.evaluateError(wX, H);
  const Vector expected = Vector1(24.0);

  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(1, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  Matrix expectedH(1, 3);
  expectedH << -4.0, -6.0, -12.0;
  EXPECT(assert_equal(expectedH, H));

  const Factor exactFactor(kKey, wL, 7.0, model);
  EXPECT(assert_equal(Vector1::Zero(), exactFactor.evaluateError(wX)));

  Values values;
  values.insert(kKey, wX);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that dynamically sized 3D landmarks are accepted after validation.
TEST(KnownLandmarkSquaredRangeFactor, AcceptsDynamicLandmark3D) {
  Vector wL(3);
  wL << 3.0, 5.0, 9.0;
  const Point wX(1.0, 2.0, 3.0);
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, 5.0, model);
  EXPECT(assert_equal(Vector1(24.0), factor.evaluateError(wX)));
}

// Verifies that short and long 3D landmark vectors throw before Eigen
// conversion.
TEST(KnownLandmarkSquaredRangeFactor, RejectsWrongLandmarkDimension3D) {
  const auto model = noiseModel::Unit::Create(1);
  const Vector tooShort = Vector::Zero(2);
  const Vector tooLong = Vector::Zero(4);

  CHECK_EXCEPTION(Factor(kKey, tooShort, 5.0, model), std::invalid_argument);
  CHECK_EXCEPTION(Factor(kKey, tooLong, 5.0, model), std::invalid_argument);
}

// Verifies cloning preserves factor data and equality detects changed data.
TEST(KnownLandmarkSquaredRangeFactor, CloneAndEquals) {
  const Point wL(3.0, 5.0, 9.0);
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, 5.0, model);
  const auto clone = factor.clone();

  EXPECT(factor.equals(*clone));

  Point shiftedLandmark = wL;
  shiftedLandmark(0) += 0.1;
  EXPECT(!factor.equals(Factor(kKey, shiftedLandmark, 5.0, model)));

  EXPECT(!factor.equals(Factor(kKey, wL, 5.1, model)));

  NonlinearFactorGraph graph;
  graph.push_back(clone);
  const NonlinearFactorGraph graphClone = graph.clone();
  EXPECT(std::dynamic_pointer_cast<Factor>(graphClone.at(0)) != nullptr);
}

}  // namespace point3_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
