/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInertialNavFactor.cpp
 * @brief   Unit test for the InertialNavFactor
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/functors.h>
#include <gtsam/navigation/AggregateImuReadings.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

static const double kDt = 0.1;

/* ************************************************************************* */
TEST(AggregateImuReadings, CorrectWithExpmapDerivative1) {
  Matrix aH1, aH2;
  boost::function<Vector3(const Vector3&, const Vector3&)> f = boost::bind(
      correctWithExpmapDerivative, _1, _2, boost::none, boost::none);
  for (Vector3 omega : {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)}) {
    for (Vector3 theta :
         {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)}) {
      Vector3 expected = Rot3::ExpmapDerivative(omega) * theta;
      EXPECT(assert_equal(expected, correctWithExpmapDerivative(omega, theta)));
      EXPECT(assert_equal(expected,
                          correctWithExpmapDerivative(omega, theta, aH1, aH2)));
      EXPECT(assert_equal(numericalDerivative21(f, omega, theta), aH1));
      EXPECT(assert_equal(numericalDerivative22(f, omega, theta), aH2));
    }
  }
}

/* ************************************************************************* */
TEST(AggregateImuReadings, CorrectWithExpmapDerivative2) {
  Matrix aH1, aH2;
  boost::function<Vector3(const Vector3&, const Vector3&)> f = boost::bind(
      correctWithExpmapDerivative, _1, _2, boost::none, boost::none);
  const Vector3 omega(0, 0, 0);
  for (Vector3 theta : {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)}) {
    Vector3 expected = Rot3::ExpmapDerivative(omega) * theta;
    EXPECT(assert_equal(expected, correctWithExpmapDerivative(omega, theta)));
    EXPECT(assert_equal(expected,
                        correctWithExpmapDerivative(omega, theta, aH1, aH2)));
    EXPECT(assert_equal(numericalDerivative21(f, omega, theta), aH1));
    EXPECT(assert_equal(numericalDerivative22(f, omega, theta), aH2));
  }
}

/* ************************************************************************* */
TEST(AggregateImuReadings, CorrectWithExpmapDerivative3) {
  Matrix aH1, aH2;
  boost::function<Vector3(const Vector3&, const Vector3&)> f = boost::bind(
      correctWithExpmapDerivative, _1, _2, boost::none, boost::none);
  const Vector3 omega(0.1, 0.2, 0.3), theta(0.4, 0.3, 0.2);
  Vector3 expected = Rot3::ExpmapDerivative(omega) * theta;
  EXPECT(assert_equal(expected, correctWithExpmapDerivative(omega, theta)));
  EXPECT(assert_equal(expected,
                      correctWithExpmapDerivative(omega, theta, aH1, aH2)));
  EXPECT(assert_equal(numericalDerivative21(f, omega, theta), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, omega, theta), aH2));
}

/* ************************************************************************* */
TEST(AggregateImuReadings, PredictAngularVelocity1) {
  Matrix aH1, aH2;
  PredictAngularVelocity functor(kDt);
  boost::function<Vector3(const Vector3&, const Vector3&)> f =
      boost::bind(functor, _1, _2, boost::none, boost::none);
  const Vector3 theta(0, 0, 0), theta_plus(0.4, 0.3, 0.2);
  EXPECT(assert_equal(Vector3(4, 3, 2), functor(theta, theta_plus, aH1, aH2)));
  EXPECT(assert_equal(numericalDerivative21(f, theta, theta_plus), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, theta, theta_plus), aH2));
}

/* ************************************************************************* */
TEST(AggregateImuReadings, PredictAngularVelocity2) {
  Matrix aH1, aH2;
  PredictAngularVelocity functor(kDt);
  boost::function<Vector3(const Vector3&, const Vector3&)> f =
      boost::bind(functor, _1, _2, boost::none, boost::none);
  const Vector3 theta(0.1, 0.2, 0.3), theta_plus(0.4, 0.3, 0.2);
  EXPECT(
      assert_equal(Vector3(Rot3::ExpmapDerivative(theta) * Vector3(3, 1, -1)),
                   functor(theta, theta_plus, aH1, aH2), 1e-5));
  EXPECT(assert_equal(numericalDerivative21(f, theta, theta_plus), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, theta, theta_plus), aH2));
}

/* ************************************************************************* */
TEST(AggregateImuReadings, AverageVelocity) {
  Matrix aH1, aH2;
  boost::function<Vector3(const Vector3&, const Vector3&)> f =
      boost::bind(averageVelocity, _1, _2, boost::none, boost::none);
  const Vector3 v(1, 2, 3), v_plus(3, 2, 1);
  EXPECT(assert_equal(Vector3(2, 2, 2), averageVelocity(v, v_plus, aH1, aH2)));
  EXPECT(assert_equal(numericalDerivative21(f, v, v_plus), aH1));
  EXPECT(assert_equal(numericalDerivative22(f, v, v_plus), aH2));
}

/* ************************************************************************* */
TEST(AggregateImuReadings, PositionDefect) {
  Matrix aH1, aH2, aH3;
  PositionDefect functor(kDt);
  boost::function<Vector3(const Vector3&, const Vector3&, const Vector3&)> f =
      boost::bind(functor, _1, _2, _3, boost::none, boost::none, boost::none);
  const Vector3 pos(1, 2, 3), pos_plus(2, 4, 6);
  const Vector3 avg(10, 20, 30);
  EXPECT(assert_equal(Vector3(0, 0, 0),
                      functor(pos, pos_plus, avg, aH1, aH2, aH3)));
  EXPECT(assert_equal(numericalDerivative31(f, pos, pos_plus, avg), aH1));
  EXPECT(assert_equal(numericalDerivative32(f, pos, pos_plus, avg), aH2));
  EXPECT(assert_equal(numericalDerivative33(f, pos, pos_plus, avg), aH3));
}

/* ************************************************************************* */
TEST(AggregateImuReadings, PredictAcceleration1) {
  Matrix aH1, aH2, aH3;
  PredictAcceleration functor(kDt);
  boost::function<Vector3(const Vector3&, const Vector3&, const Vector3&)> f =
      boost::bind(functor, _1, _2, _3, boost::none, boost::none, boost::none);
  const Vector3 vel(1, 2, 3), vel_plus(2, 4, 6);
  const Vector3 theta(0, 0, 0);
  EXPECT(assert_equal(Vector3(10, 20, 30),
                      functor(vel, vel_plus, theta, aH1, aH2, aH3)));
  EXPECT(assert_equal(numericalDerivative31(f, vel, vel_plus, theta), aH1));
  EXPECT(assert_equal(numericalDerivative32(f, vel, vel_plus, theta), aH2));
  EXPECT(assert_equal(numericalDerivative33(f, vel, vel_plus, theta), aH3));
}

/* ************************************************************************* */
TEST(AggregateImuReadings, PredictAcceleration2) {
  Matrix aH1, aH2, aH3;
  PredictAcceleration functor(kDt);
  boost::function<Vector3(const Vector3&, const Vector3&, const Vector3&)> f =
      boost::bind(functor, _1, _2, _3, boost::none, boost::none, boost::none);
  const Vector3 vel(1, 2, 3), vel_plus(2, 4, 6);
  const Vector3 theta(0.1, 0.2, 0.3);
  EXPECT(assert_equal(Vector3(10, 20, 30),
                      functor(vel, vel_plus, theta, aH1, aH2, aH3)));
  EXPECT(assert_equal(numericalDerivative31(f, vel, vel_plus, theta), aH1));
  EXPECT(assert_equal(numericalDerivative32(f, vel, vel_plus, theta), aH2));
  EXPECT(assert_equal(numericalDerivative33(f, vel, vel_plus, theta), aH3));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
