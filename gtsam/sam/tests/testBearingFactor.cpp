/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testBearingFactor.cpp
 *  @brief Unit tests for BearingFactor Class
 *  @author Frank Dellaert
 *  @date July 2015
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sam/BearingFactor.h>

using namespace std;
using namespace gtsam;

namespace {
Key poseKey(1);
Key pointKey(2);

typedef BearingFactor<Pose2, Point2> BearingFactor2D;
double measurement2D(10.0);
static SharedNoiseModel model2D(noiseModel::Isotropic::Sigma(1, 0.5));
BearingFactor2D factor2D(poseKey, pointKey, measurement2D, model2D);

typedef BearingFactor<Pose3, Point3> BearingFactor3D;
Unit3 measurement3D = Pose3().bearing(Point3(1, 0, 0));  // has to match values!
static SharedNoiseModel model3D(noiseModel::Isotropic::Sigma(2, 0.5));
BearingFactor3D factor3D(poseKey, pointKey, measurement3D, model3D);
}  // namespace

/* ************************************************************************* */
TEST(BearingFactor, 2D) {
  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose2(1.0, 2.0, 0.57));
  values.insert(pointKey, Point2(-4.0, 11.0));

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor2D, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// TODO(frank): this test is disabled (for now) because the macros below are
// incompatible with the Unit3 localCoordinates. The issue is the following:
// For factors, we want to use Local(value, measured), because we need the error
// to be expressed in the tangent space of value. This surfaced in the Unit3 case
// where the tangent space can be radically different from one value to the next.
// For derivatives, we want Local(constant, varying), because we need a derivative
// in a constant tangent space. But since the macros below call whitenedError
// which calls Local(value,measured), we actually call the reverse. This does not
// matter for types with a commutative Local, but matters a lot for Unit3.
// More thinking needed about what the right thing is, here...
//TEST(BearingFactor, 3D) {
//  // Serialize the factor
//  std::string serialized = serializeXML(factor3D);
//
//  // And de-serialize it
//  BearingFactor3D factor;
//  deserializeXML(serialized, factor);
//
//  // Set the linearization point
//  Values values;
//  values.insert(poseKey, Pose3());
//  values.insert(pointKey, Point3(1, 0, 0));
//
//  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
//}

/* ************************************************************************* */
namespace binary_linearization {

// Calls the base implementation explicitly to obtain a generic JacobianFactor,
// then checks that the normal call returns an equal BinaryJacobianFactor<M,N1,N2>,
// where M is the residual dimension and N1/N2 are variable tangent dimensions.
TEST(BearingFactor, BinaryLinearization) {
  // A Pose2 has three tangent dimensions, a Point2 has two, and a Rot2
  // bearing has one. The optimized result should therefore have shape <1,3,2>.
  const Values values2D{{poseKey, genericValue(Pose2(1.0, 2.0, 0.57))},
                        {pointKey, genericValue(Point2(-4.0, 11.0))}};
  const auto generic2D = factor2D.NoiseModelFactor::linearize(values2D);
  const auto optimized2D = factor2D.linearize(values2D);
  const bool isBinary2D = static_cast<bool>(
      std::dynamic_pointer_cast<BinaryJacobianFactor<1, 3, 2>>(optimized2D));
  CHECK(isBinary2D);
  EXPECT(assert_equal(*generic2D, *optimized2D, 1e-9));

  // A Unit3 bearing has two local dimensions, while Pose3 and Point3 have six
  // and three. The base-qualified call bypasses the optimized override and
  // supplies the reference whitened Jacobians and right-hand side.
  const Values values3D{{poseKey, genericValue(Pose3())},
                        {pointKey, genericValue(Point3(1.0, 0.0, 0.0))}};
  const auto generic3D = factor3D.NoiseModelFactor::linearize(values3D);
  const auto optimized3D = factor3D.linearize(values3D);
  const bool isBinary3D = static_cast<bool>(
      std::dynamic_pointer_cast<BinaryJacobianFactor<2, 6, 3>>(optimized3D));
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
