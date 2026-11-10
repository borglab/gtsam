/* ----------------------------------------------------------------------------
 *
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */
/**
 * @file    testWnoaAugmentedPoint.cpp
 * @brief   WNOA motion factor tests for AugmentedPoint.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaFactor.h>
#include <gtsam/nonlinear/WnoaInterpolator.h>
#include <gtsam/nonlinear/WnoaInterpFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam_unstable/geometry/AugmentedPoint.h>
#include <gtsam_unstable/slam/KnownLandmarkSquaredRangeFactor.h>

#include <array>
#include <set>
#include <vector>

using namespace gtsam;
using symbol_shorthand::P;
using symbol_shorthand::V;

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, ConstantVelocity) {
  using Point = AugmentedPoint<3>;
  const double dt = 0.2;
  const Point p0(Vector3(1.0, -2.0, 3.0));
  const Vector3 velocity(0.5, -0.25, 0.75);
  const Point p1 = p0 + dt * velocity;

  WnoaMotionFactor<Point> factor(P(0), V(0), P(1), V(1), dt,
                                 Vector3::Ones());
  const Vector actual = factor.evaluateError(p0, velocity, p1, velocity);
  const Vector6 expected = Vector6::Zero();
  CHECK(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, Jacobians) {
  using Point = AugmentedPoint<3>;
  const Point p0(Vector3(1.0, -2.0, 3.0));
  const Point p1(Vector3(1.3, -1.7, 2.8));
  const Vector3 v0(0.5, -0.25, 0.75);
  const Vector3 v1(0.25, 0.1, 0.5);

  WnoaMotionFactor<Point> factor(P(0), V(0), P(1), V(1), 0.2,
                                 Vector3::Ones());
  Values values;
  values.insert(P(0), p0);
  values.insert(V(0), v0);
  values.insert(P(1), p1);
  values.insert(V(1), v1);

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, InterpolationConstantVelocity) {
  using Point = AugmentedPoint<3>;
  const double dt = 0.2;
  const Point p0(Vector3(1.0, -2.0, 3.0));
  const Vector3 velocity(0.5, -0.25, 0.75);
  const Point p1 = p0 + dt * velocity;
  Interpolator<Point> interpolator(Vector3::Ones());

  for (double ratio : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    const auto result = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point>(p0, velocity, 0.0),
        TimestampedPoseVelocity<Point>(p1, velocity, dt), ratio * dt);
    const Point expected = p0 + ratio * dt * velocity;
    CHECK(assert_equal(expected, result.pose, 1e-9));
    CHECK(assert_equal(velocity, result.vel, 1e-9));
  }
}

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, InterpolationJacobians) {
  using Point = AugmentedPoint<3>;
  const Point p0(Vector3(1.0, -2.0, 3.0));
  const Point p1(Vector3(1.3, -1.7, 2.8));
  const Vector3 v0(0.5, -0.25, 0.75);
  const Vector3 v1(0.25, 0.1, 0.5);
  const double dt = 0.2, tau = 0.08;
  Interpolator<Point> interpolator(Vector3::Ones());

  std::vector<Matrix> H(8);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point>(p0, v0, 0.0),
      TimestampedPoseVelocity<Point>(p1, v1, dt), tau, &H);

  auto position = [&](const Point& a, const Vector3& b, const Point& c,
                      const Vector3& d) -> Vector3 {
    const auto result = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point>(a, b, 0.0),
        TimestampedPoseVelocity<Point>(c, d, dt), tau);
    return result.pose.vector();
  };
  auto velocity = [&](const Point& a, const Vector3& b, const Point& c,
                      const Vector3& d) -> Vector3 {
    const auto result = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point>(a, b, 0.0),
        TimestampedPoseVelocity<Point>(c, d, dt), tau);
    return result.vel;
  };

  const double delta = 1e-6, tol = 1e-5;
  EXPECT(assert_equal(numericalDerivative41<Vector3, Point, Vector3, Point,
                                           Vector3>(position, p0, v0, p1, v1,
                                                    delta),
                      H[0], tol));
  EXPECT(assert_equal(numericalDerivative42<Vector3, Point, Vector3, Point,
                                           Vector3>(position, p0, v0, p1, v1,
                                                    delta),
                      H[1], tol));
  EXPECT(assert_equal(numericalDerivative43<Vector3, Point, Vector3, Point,
                                           Vector3>(position, p0, v0, p1, v1,
                                                    delta),
                      H[2], tol));
  EXPECT(assert_equal(numericalDerivative44<Vector3, Point, Vector3, Point,
                                           Vector3>(position, p0, v0, p1, v1,
                                                    delta),
                      H[3], tol));
  EXPECT(assert_equal(numericalDerivative41<Vector3, Point, Vector3, Point,
                                           Vector3>(velocity, p0, v0, p1, v1,
                                                    delta),
                      H[4], tol));
  EXPECT(assert_equal(numericalDerivative42<Vector3, Point, Vector3, Point,
                                           Vector3>(velocity, p0, v0, p1, v1,
                                                    delta),
                      H[5], tol));
  EXPECT(assert_equal(numericalDerivative43<Vector3, Point, Vector3, Point,
                                           Vector3>(velocity, p0, v0, p1, v1,
                                                    delta),
                      H[6], tol));
  EXPECT(assert_equal(numericalDerivative44<Vector3, Point, Vector3, Point,
                                           Vector3>(velocity, p0, v0, p1, v1,
                                                    delta),
                      H[7], tol));
}

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, InterpolatedSquaredRange) {
  using Point = AugmentedPoint<3>;
  const double dt = 0.2;
  const Point p0(Vector3(1.0, 0.0, 0.0));
  const Vector3 velocity(1.0, 0.0, 0.0);
  const Point p1 = p0 + dt * velocity;

  const std::set<StateData> estimatedStates = {
      StateData(P(0), V(0), 0.0), StateData(P(1), V(1), dt)};
  const std::set<StateData> interpolatedStates = {
      StateData(P(2), V(2), 0.08)};

  auto range = std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
      P(2), Vector3::Zero().eval(), 1.0,
      noiseModel::Isotropic::Sigma(1, 0.1));
  WnoaInterpFactor<Point> factor(range, estimatedStates, interpolatedStates,
                                 Vector3::Ones(), true);

  Values values;
  values.insert(P(0), p0);
  values.insert(V(0), velocity);
  values.insert(P(1), p1);
  values.insert(V(1), velocity);

  CHECK(factor.keys().size() == 4);
  const Vector1 expected(1.08 * 1.08 - 1.0);
  CHECK(assert_equal(expected, factor.unwhitenedError(values), 1e-9));
}

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, InterpolatedSquaredRangeJacobians) {
  using Point = AugmentedPoint<3>;
  const Point p0(Vector3(1.0, -2.0, 3.0));
  const Point p1(Vector3(1.3, -1.7, 2.8));
  const Vector3 v0(0.5, -0.25, 0.75);
  const Vector3 v1(0.25, 0.1, 0.5);

  const std::set<StateData> estimatedStates = {
      StateData(P(0), V(0), 0.0), StateData(P(1), V(1), 0.2)};
  const std::set<StateData> interpolatedStates = {
      StateData(P(2), V(2), 0.08)};

  auto range = std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
      P(2), Vector3(2.0, 0.5, -1.0), 2.5,
      noiseModel::Isotropic::Sigma(1, 0.1));
  WnoaInterpFactor<Point> factor(range, estimatedStates, interpolatedStates,
                                 Vector3::Ones(), true);

  Values values;
  values.insert(P(0), p0);
  values.insert(V(0), v0);
  values.insert(P(1), p1);
  values.insert(V(1), v1);

  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-6, 1e-5);
}

/* ************************************************************************* */
TEST(WnoaAugmentedPoint, RangeOnlyOptimization) {
  using Point = AugmentedPoint<3>;
  const Vector3 p0(0.6, -0.2, 0.7);
  const Vector3 velocity(0.35, 0.1, -0.15);
  const Vector3 q = Vector3::Constant(0.2);
  const std::set<StateData> knots = {
      StateData(P(0), V(0), 0.0), StateData(P(1), V(1), 1.0),
      StateData(P(2), V(2), 2.0)};
  const std::array<Vector3, 4> landmarks = {
      Vector3(0.0, 0.0, 0.0), Vector3(3.0, 0.0, 0.0),
      Vector3(0.0, 3.0, 0.0), Vector3(0.0, 0.0, 3.0)};
  const std::array<double, 4> measurementTimes = {0.25, 0.75, 1.25, 1.75};

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Point>>(
      P(0), Point(p0), noiseModel::Isotropic::Sigma(3, 0.05));
  for (size_t k = 0; k < 2; ++k) {
    graph.emplace_shared<WnoaMotionFactor<Point>>(
        StateData(P(k), V(k), double(k)),
        StateData(P(k + 1), V(k + 1), double(k + 1)), q);
  }

  const auto rangeNoise = noiseModel::Isotropic::Sigma(1, 0.02);
  for (size_t j = 0; j < measurementTimes.size(); ++j) {
    const double t = measurementTimes[j];
    const Vector3 p = p0 + t * velocity;
    const StateData interp(P(10 + j), V(10 + j), t);
    for (const Vector3& landmark : landmarks) {
      const double range = (p - landmark).norm();
      auto inner = std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
          interp.pose, landmark, range, rangeNoise);
      graph.emplace_shared<WnoaInterpFactor<Point>>(
          inner, knots, std::set<StateData>{interp}, q, true);
    }
  }

  Values initial;
  const Vector3 positionOffset(0.12, -0.08, 0.1);
  const Vector3 velocityOffset(-0.08, 0.06, 0.1);
  for (size_t k = 0; k < 3; ++k) {
    const Vector3 p = p0 + double(k) * velocity;
    initial.insert(P(k), Point(p + positionOffset));
    initial.insert(V(k), velocity + velocityOffset);
  }

  const double initialError = graph.error(initial);
  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  const double finalError = graph.error(result);

  CHECK(initialError > 0.0);
  CHECK(finalError < initialError);
  CHECK(finalError < 1e-4);
  CHECK(result.size() == 6);  // No states added at the range timestamps.
  for (size_t k = 0; k < 3; ++k) {
    const Vector3 expected = p0 + double(k) * velocity;
    CHECK((result.at<Point>(P(k)).vector() - expected).norm() < 1e-2);
    CHECK((result.at<Vector3>(V(k)) - velocity).norm() < 1e-2);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
