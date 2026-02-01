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

#include <gtsam/sam/BearingFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/expressionTesting.h>

#include <CppUnitLite/TestHarness.h>

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
}

/* ************************************************************************* */
TEST(BearingFactor, 2D) {
  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose2(1.0, 2.0, 0.57));
  values.insert(pointKey, Point2(-4.0, 11.0));

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(factor2D.expression({poseKey, pointKey}),
                                      values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor2D, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// TODO(frank): this test is disabled (for now) because the macros below are
// incompatible with the Unit3 localCoordinates. The issue is the following:
// For factors, we want to use Local(value, measured), because we need the error
// to be expressed in the tangent space of value. This surfaced in the Unit3 case
// where the tangent space can be radically didfferent from one value to the next.
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
//  EXPECT_CORRECT_EXPRESSION_JACOBIANS(factor.expression({poseKey, pointKey}),
//                                      values, 1e-7, 1e-5);
//  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
