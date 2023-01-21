/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testRangeFactor.cpp
 *  @brief Unit tests for RangeFactor Class
 *  @author Stephen Williams
 *  @date Oct 2012
 */

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind/bind.hpp>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef RangeFactor<Pose2, Point2> RangeFactor2D;
typedef RangeFactor<Pose3, Point3> RangeFactor3D;
typedef RangeFactorWithTransform<Pose2, Point2> RangeFactorWithTransform2D;
typedef RangeFactorWithTransform<Pose3, Point3> RangeFactorWithTransform3D;

// Keys are deliberately *not* in sorted order to test that case.
namespace {
// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(1));

constexpr Key poseKey(2);
constexpr Key pointKey(1);
constexpr double measurement(10.0);

Vector factorError2D(const Pose2& pose, const Point2& point,
                     const RangeFactor2D& factor) {
  return factor.evaluateError(pose, point);
}

Vector factorError3D(const Pose3& pose, const Point3& point,
                     const RangeFactor3D& factor) {
  return factor.evaluateError(pose, point);
}

Vector factorErrorWithTransform2D(const Pose2& pose, const Point2& point,
                                  const RangeFactorWithTransform2D& factor) {
  return factor.evaluateError(pose, point);
}

Vector factorErrorWithTransform3D(const Pose3& pose, const Point3& point,
                                  const RangeFactorWithTransform3D& factor) {
  return factor.evaluateError(pose, point);
}
}  // namespace

/* ************************************************************************* */
TEST( RangeFactor, Constructor) {
  RangeFactor2D factor2D(poseKey, pointKey, measurement, model);
  RangeFactor3D factor3D(poseKey, pointKey, measurement, model);
}

/* ************************************************************************* */
TEST( RangeFactor, ConstructorWithTransform) {
  Pose2 body_P_sensor_2D(0.25, -0.10, -M_PI_2);
  Pose3 body_P_sensor_3D(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
      Point3(0.25, -0.10, 1.0));

  RangeFactorWithTransform2D factor2D(poseKey, pointKey, measurement, model,
      body_P_sensor_2D);
  KeyVector expected {2, 1};
  CHECK(factor2D.keys() == expected);
  RangeFactorWithTransform3D factor3D(poseKey, pointKey, measurement, model,
      body_P_sensor_3D);
  CHECK(factor3D.keys() == expected);
}

/* ************************************************************************* */
TEST( RangeFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  RangeFactor2D factor2D_1(poseKey, pointKey, measurement, model);
  RangeFactor2D factor2D_2(poseKey, pointKey, measurement, model);
  CHECK(assert_equal(factor2D_1, factor2D_2));

  RangeFactor3D factor3D_1(poseKey, pointKey, measurement, model);
  RangeFactor3D factor3D_2(poseKey, pointKey, measurement, model);
  CHECK(assert_equal(factor3D_1, factor3D_2));
}

/* ************************************************************************* */
TEST( RangeFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Pose2 body_P_sensor_2D(0.25, -0.10, -M_PI_2);
  Pose3 body_P_sensor_3D(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
      Point3(0.25, -0.10, 1.0));

  RangeFactorWithTransform2D factor2D_1(poseKey, pointKey, measurement, model,
      body_P_sensor_2D);
  RangeFactorWithTransform2D factor2D_2(poseKey, pointKey, measurement, model,
      body_P_sensor_2D);
  CHECK(assert_equal(factor2D_1, factor2D_2));

  RangeFactorWithTransform3D factor3D_1(poseKey, pointKey, measurement, model,
      body_P_sensor_3D);
  RangeFactorWithTransform3D factor3D_2(poseKey, pointKey, measurement, model,
      body_P_sensor_3D);
  CHECK(assert_equal(factor3D_1, factor3D_2));
}
/* ************************************************************************* */
TEST( RangeFactor, Error2D ) {
  // Create a factor
  RangeFactor2D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose2 pose(1.0, 2.0, 0.57);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the error
  Vector actualError(factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}));

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Error2DWithTransform ) {
  // Create a factor
  Pose2 body_P_sensor(0.25, -0.10, -M_PI_2);
  RangeFactorWithTransform2D factor(poseKey, pointKey, measurement, model,
      body_P_sensor);

  // Set the linearization point
  Rot2 R(0.57);
  Point2 t = Point2(1.0, 2.0) - R.rotate(body_P_sensor.translation());
  Pose2 pose(R, t);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the error
  Vector actualError(factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}));

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Error3D ) {
  // Create a factor
  RangeFactor3D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose3 pose(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0, -3.0));
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the error
  Vector actualError(factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}));

  // The expected error is ||(3.0, 9.0, 4.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Error3DWithTransform ) {
  // Create a factor
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
      Point3(0.25, -0.10, 1.0));
  RangeFactorWithTransform3D factor(poseKey, pointKey, measurement, model,
      body_P_sensor);

  // Set the linearization point
  Rot3 R = Rot3::RzRyRx(0.2, -0.3, 1.75);
  Point3 t = Point3(1.0, 2.0, -3.0) - R.rotate(body_P_sensor.translation());
  Pose3 pose(R, t);
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the error
  Vector actualError(factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}));

  // The expected error is ||(3.0, 9.0, 4.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Jacobian2D ) {
  // Create a factor
  RangeFactor2D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose2 pose(1.0, 2.0, 0.57);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, point, &H1Actual, &H2Actual);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<Vector, Pose2>(
      std::bind(&factorError2D, std::placeholders::_1, point, factor), pose);
  H2Expected = numericalDerivative11<Vector, Point2>(
      std::bind(&factorError2D, pose, std::placeholders::_1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Jacobian2DWithTransform ) {
  // Create a factor
  Pose2 body_P_sensor(0.25, -0.10, -M_PI_2);
  RangeFactorWithTransform2D factor(poseKey, pointKey, measurement, model,
      body_P_sensor);

  // Set the linearization point
  Rot2 R(0.57);
  Point2 t = Point2(1.0, 2.0) - R.rotate(body_P_sensor.translation());
  Pose2 pose(R, t);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the Jacobians
  std::vector<Matrix> actualHs(2);
  factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}, actualHs); 
  const Matrix& H1Actual = actualHs.at(0);
  const Matrix& H2Actual = actualHs.at(1);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<Vector, Pose2>(
      std::bind(&factorErrorWithTransform2D, std::placeholders::_1, point, factor), pose);
  H2Expected = numericalDerivative11<Vector, Point2>(
      std::bind(&factorErrorWithTransform2D, pose, std::placeholders::_1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Jacobian3D ) {
  // Create a factor
  RangeFactor3D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose3 pose(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0, -3.0));
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the Jacobians
  std::vector<Matrix> actualHs(2);
  factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}, actualHs); 
  const Matrix& H1Actual = actualHs.at(0);
  const Matrix& H2Actual = actualHs.at(1);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<Vector, Pose3>(
      std::bind(&factorError3D, std::placeholders::_1, point, factor), pose);
  H2Expected = numericalDerivative11<Vector, Point3>(
      std::bind(&factorError3D, pose, std::placeholders::_1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Jacobian3DWithTransform ) {
  // Create a factor
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2),
      Point3(0.25, -0.10, 1.0));
  RangeFactorWithTransform3D factor(poseKey, pointKey, measurement, model,
      body_P_sensor);

  // Set the linearization point
  Rot3 R = Rot3::RzRyRx(0.2, -0.3, 1.75);
  Point3 t = Point3(1.0, 2.0, -3.0) - R.rotate(body_P_sensor.translation());
  Pose3 pose(R, t);
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the Jacobians
  std::vector<Matrix> actualHs(2);
  factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}, actualHs); 
  const Matrix& H1Actual = actualHs.at(0);
  const Matrix& H2Actual = actualHs.at(1);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<Vector, Pose3>(
      std::bind(&factorErrorWithTransform3D, std::placeholders::_1, point, factor), pose);
  H2Expected = numericalDerivative11<Vector, Point3>(
      std::bind(&factorErrorWithTransform3D, pose, std::placeholders::_1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
// Do a test with Point2
TEST(RangeFactor, Point2) {
  // Create a factor
  RangeFactor<Point2> factor(11, 22, measurement, model);

  // Set the linearization point
  Point2 p11(1.0, 2.0), p22(-4.0, 11.0);

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  Values values {{11, genericValue(p11)}, {22, genericValue(p22)}};
  CHECK(assert_equal(expectedError, factor.unwhitenedError(values), 1e-9));
}

/* ************************************************************************* */
// Do a test with Point3
TEST(RangeFactor, Point3) {
  // Create a factor
  RangeFactor<Point3> factor(11, 22, measurement, model);

  // Set the linearization point
  Point3 p11(1.0, 2.0, 0.0), p22(-4.0, 11.0, 0);

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  Values values {{11, genericValue(p11)}, {22, genericValue(p22)}};
  CHECK(assert_equal(expectedError, factor.unwhitenedError(values), 1e-9));
}

/* ************************************************************************* */
// Do tests with PinholeCamera<Cal3_S2>
TEST( RangeFactor, Camera) {
  using Camera = PinholeCamera<Cal3_S2>;

  RangeFactor<Camera, Point3> factor1(poseKey, pointKey, measurement, model);
  RangeFactor<Camera, Pose3>  factor2(poseKey, pointKey, measurement, model);
  RangeFactor<Camera, Camera> factor3(poseKey, pointKey, measurement, model);
}

/* ************************************************************************* */
// Do a test with non GTSAM types

namespace gtsam {
template <>
struct Range<Vector4, Vector4> {
  typedef double result_type;
  double operator()(const Vector4& v1, const Vector4& v2,
                    OptionalJacobian<1, 4> H1, OptionalJacobian<1, 4> H2) {
    return (v2 - v1).norm();
    // derivatives not implemented
  }
};
}  // namespace gtsam

TEST(RangeFactor, NonGTSAM) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  RangeFactor<Vector4> factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Vector4 pose(1.0, 2.0, 00, 0);
  Vector4 point(-4.0, 11.0, 0, 0);

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, factor.unwhitenedError({{poseKey, genericValue(pose)}, {pointKey, genericValue(point)}}), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
