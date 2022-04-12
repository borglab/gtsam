/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSerializationSam.cpp
 *  @brief All serialization test in this directory
 *  @author Frank Dellaert
 *  @date February 2022
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>

using namespace std;
using namespace gtsam;

namespace {
Key poseKey(1);
Key pointKey(2);
constexpr double rangeMmeasurement(10.0);
}  // namespace

/* ************************************************************************* */
// Export Noisemodels
// See http://www.boost.org/doc/libs/1_32_0/libs/serialization/doc/special.html
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic);
BOOST_CLASS_EXPORT(gtsam::noiseModel::Unit);

/* ************************************************************************* */
TEST(SerializationSam, BearingFactor2D) {
  using BearingFactor2D = BearingFactor<Pose2, Point2>;
  double measurement2D(10.0);
  static SharedNoiseModel model2D(noiseModel::Isotropic::Sigma(1, 0.5));
  BearingFactor2D factor2D(poseKey, pointKey, measurement2D, model2D);
  EXPECT(serializationTestHelpers::equalsObj(factor2D));
  EXPECT(serializationTestHelpers::equalsXML(factor2D));
  EXPECT(serializationTestHelpers::equalsBinary(factor2D));
}

/* ************************************************************************* */
TEST(SerializationSam, BearingFactor3D) {
  using BearingFactor3D = BearingFactor<Pose3, Point3>;
  Unit3 measurement3D =
      Pose3().bearing(Point3(1, 0, 0));  // has to match values!
  static SharedNoiseModel model3D(noiseModel::Isotropic::Sigma(2, 0.5));
  BearingFactor3D factor3D(poseKey, pointKey, measurement3D, model3D);
  EXPECT(serializationTestHelpers::equalsObj(factor3D));
  EXPECT(serializationTestHelpers::equalsXML(factor3D));
  EXPECT(serializationTestHelpers::equalsBinary(factor3D));
}

/* ************************************************************************* */
namespace {
static SharedNoiseModel rangeNoiseModel(noiseModel::Unit::Create(1));
}

TEST(SerializationSam, RangeFactor2D) {
  using RangeFactor2D = RangeFactor<Pose2, Point2>;
  RangeFactor2D factor2D(poseKey, pointKey, rangeMmeasurement, rangeNoiseModel);
  EXPECT(serializationTestHelpers::equalsObj(factor2D));
  EXPECT(serializationTestHelpers::equalsXML(factor2D));
  EXPECT(serializationTestHelpers::equalsBinary(factor2D));
}

/* ************************************************************************* */
TEST(SerializationSam, RangeFactor3D) {
  using RangeFactor3D = RangeFactor<Pose3, Point3>;
  RangeFactor3D factor3D(poseKey, pointKey, rangeMmeasurement, rangeNoiseModel);
  EXPECT(serializationTestHelpers::equalsObj(factor3D));
  EXPECT(serializationTestHelpers::equalsXML(factor3D));
  EXPECT(serializationTestHelpers::equalsBinary(factor3D));
}

/* ************************************************************************* */
TEST(RangeFactor, EqualsAfterDeserializing) {
  // Check that the same results are obtained after deserializing:
  Pose3 body_P_sensor_3D(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
                         Point3(0.25, -0.10, 1.0));
  RangeFactorWithTransform<Pose3, Point3> factor3D_1(
      poseKey, pointKey, rangeMmeasurement, rangeNoiseModel, body_P_sensor_3D),
      factor3D_2;

  // check with Equal() trait:
  gtsam::serializationTestHelpers::roundtripXML(factor3D_1, factor3D_2);
  CHECK(assert_equal(factor3D_1, factor3D_2));

  const Pose3 pose(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0, -3.0));
  const Point3 point(-2.0, 11.0, 1.0);
  const Values values = {{poseKey, genericValue(pose)},
                         {pointKey, genericValue(point)}};

  const Vector error_1 = factor3D_1.unwhitenedError(values);
  const Vector error_2 = factor3D_2.unwhitenedError(values);
  CHECK(assert_equal(error_1, error_2));
}

/* ************************************************************************* */
TEST(BearingRangeFactor, Serialization2D) {
  using BearingRangeFactor2D = BearingRangeFactor<Pose2, Point2>;
  static SharedNoiseModel model2D(noiseModel::Isotropic::Sigma(2, 0.5));
  BearingRangeFactor2D factor2D(poseKey, pointKey, 1, 2, model2D);

  EXPECT(serializationTestHelpers::equalsObj(factor2D));
  EXPECT(serializationTestHelpers::equalsXML(factor2D));
  EXPECT(serializationTestHelpers::equalsBinary(factor2D));
}

/* ************************************************************************* */
TEST(BearingRangeFactor, Serialization3D) {
  using BearingRangeFactor3D = BearingRangeFactor<Pose3, Point3>;
  static SharedNoiseModel model3D(noiseModel::Isotropic::Sigma(3, 0.5));
  BearingRangeFactor3D factor3D(poseKey, pointKey,
                                Pose3().bearing(Point3(1, 0, 0)), 1, model3D);
  EXPECT(serializationTestHelpers::equalsObj(factor3D));
  EXPECT(serializationTestHelpers::equalsXML(factor3D));
  EXPECT(serializationTestHelpers::equalsBinary(factor3D));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
