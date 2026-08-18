/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testBearingRangeFactor.cpp
 *  @brief Unit tests for BearingRangeFactor Class
 *  @author Frank Dellaert
 *  @date July 2015
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sam/BearingRangeFactor.h>

using namespace std;
using namespace gtsam;

namespace {
Key poseKey(1);
Key pointKey(2);
}  // namespace

/* ************************************************************************* */
TEST(BearingRangeFactor, 2D) {
  typedef BearingRangeFactor<Pose2, Point2> BearingRangeFactor2D;
  SharedNoiseModel model2D(noiseModel::Isotropic::Sigma(2, 0.5));
  BearingRangeFactor2D factor2D(poseKey, pointKey, 1, 2, model2D);

  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose2(1.0, 2.0, 0.57));
  values.insert(pointKey, Point2(-4.0, 11.0));

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor2D, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BearingRangeFactor, 3D) {
  typedef BearingRangeFactor<Pose3, Point3> BearingRangeFactor3D;
  SharedNoiseModel model3D(noiseModel::Isotropic::Sigma(3, 0.5));

  const Unit3 bearing = Pose3().bearing(Point3(1, 0, 0));
  const double range = 1.0;
  BearingRangeFactor3D factor3D(poseKey, pointKey, bearing, range, model3D);

  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose3());
  values.insert(pointKey, Point3(1, 0, 0));

  // Check that the error is zero at the linearization point
  Vector actualError = factor3D.unwhitenedError(values);
  EXPECT(assert_equal(Vector::Zero(actualError.size()), actualError, 1e-9));

  // TODO(frank): this test is disabled (for now) because the macros below are
  // incompatible with the Unit3 localCoordinates. See testBearingFactor...
  // EXPECT_CORRECT_FACTOR_JACOBIANS(factor3D, values, 1e-7, 1e-5);
}  // namespace

/* ************************************************************************* */
namespace binary_linearization {

// Calls the base implementation explicitly to obtain a generic JacobianFactor,
// then checks that the normal call returns an equal FixedJacobianFactor<M,N1,N2>,
// where M is the residual dimension and N1/N2 are variable tangent dimensions.
TEST(BearingRangeFactor, BinaryLinearization) {
  // The 2D measurement stacks a one-dimensional Rot2 bearing and one range,
  // giving two residual rows for Pose2 (3 DOF) and Point2 (2 DOF).
  const Values values2D{{poseKey, genericValue(Pose2(1.0, 2.0, 0.57))},
                        {pointKey, genericValue(Point2(-4.0, 11.0))}};
  const BearingRangeFactor<Pose2, Point2> factor2D(
      poseKey, pointKey, Rot2::fromAngle(0.2), 3.0,
      noiseModel::Diagonal::Sigmas(Vector2{0.5, 0.8}));
  const auto generic2D = factor2D.NoiseModelFactor::linearize(values2D);
  const auto optimized2D = factor2D.linearize(values2D);
  const bool isBinary2D = static_cast<bool>(
      std::dynamic_pointer_cast<FixedJacobianFactor<2, 3, 2>>(optimized2D));
  CHECK(isBinary2D);
  EXPECT(assert_equal(*generic2D, *optimized2D, 1e-9));

  // The 3D measurement stacks a two-dimensional Unit3 bearing and one range,
  // giving three residual rows for Pose3 (6 DOF) and Point3 (3 DOF). The
  // base-qualified call provides the generic numerical reference.
  const Values values3D{{poseKey, genericValue(Pose3())},
                        {pointKey, genericValue(Point3(1.0, 0.0, 0.0))}};
  const BearingRangeFactor<Pose3, Point3> factor3D(
      poseKey, pointKey, Pose3().bearing(Point3(1.0, 0.0, 0.0)), 1.0,
      noiseModel::Isotropic::Sigma(3, 0.5));
  const auto generic3D = factor3D.NoiseModelFactor::linearize(values3D);
  const auto optimized3D = factor3D.linearize(values3D);
  const bool isBinary3D = static_cast<bool>(
      std::dynamic_pointer_cast<FixedJacobianFactor<3, 6, 3>>(optimized3D));
  CHECK(isBinary3D);
  EXPECT(assert_equal(*generic3D, *optimized3D, 1e-9));
}

}  // namespace binary_linearization
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
