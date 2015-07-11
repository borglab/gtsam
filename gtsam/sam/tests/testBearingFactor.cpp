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
#include <gtsam/base/serialization.h>
#include <gtsam/base/serializationTestHelpers.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(1, 0.5));

Key poseKey(1);
Key pointKey(2);

typedef BearingFactor<Pose2, Point2> BearingFactor2D;
double measurement2D(10.0);
BearingFactor2D factor2D(poseKey, pointKey, measurement2D, model);
GTSAM_CONCEPT_TESTABLE_INST(BearingFactor2D)

typedef BearingFactor<Pose3, Point3, Unit3> BearingFactor3D;

/* ************************************************************************* */
// Export Noisemodels
// See http://www.boost.org/doc/libs/1_32_0/libs/serialization/doc/special.html
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic);
//BOOST_CLASS_EXPORT(BearingFactor2D);
//BOOST_CLASS_EXPORT(ExpressionFactor<Rot2>);
//BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<Rot2>, "ExpressionFactorRot2")

/* ************************************************************************* */
TEST(BearingFactor, Serialization2D) {
  EXPECT(serializationTestHelpers::equalsObj<BearingFactor2D>(factor2D));
  EXPECT(serializationTestHelpers::equalsXML<BearingFactor2D>(factor2D));
  EXPECT(serializationTestHelpers::equalsBinary<BearingFactor2D>(factor2D));
}

/* ************************************************************************* */
TEST(BearingFactor, 2D) {
  // Serialize the factor
  std::string serialized = serializeXML(factor2D);

  // And de-serialize it
  BearingFactor2D factor;
  deserializeXML(serialized, factor);

  // Set the linearization point
  Values values;
  values.insert(poseKey, Pose2(1.0, 2.0, 0.57));
  values.insert(pointKey, Point2(-4.0, 11.0));

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

/* ************************************************************************* */
// TODO(frank): this is broken: (non-existing) Pose3::bearing should yield a
// Unit3
// TEST(BearingFactor, 3D) {
//  // Create a factor
//  Rot3 measurement;
//  BearingFactor<Pose3, Point3> factor(poseKey, pointKey, measurement, model);
//
//  // Set the linearization point
//  Values values;
//  values.insert(poseKey, Pose3(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0,
//  -3.0)));
//  values.insert(pointKey, Point3(-2.0, 11.0, 1.0));
//
//  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
