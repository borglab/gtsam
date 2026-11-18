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
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
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
#include <gtsam_unstable/nonlinear/WnoaAugmentedPoint.h>
#include <gtsam_unstable/slam/KnownLandmarkSquaredRangeFactor.h>

#include <algorithm>
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
namespace qcqp_fixture {

using Point = AugmentedPoint<3>;

const Vector3 kPsd = Vector3::Constant(0.2);
const double kDt = 0.2;

// Knot states of one interval, as nonlinear Values.
Values intervalValues() {
  Values values;
  values.insert(P(0), Point(Vector3(1.0, -2.0, 3.0)));
  values.insert(V(0), Vector3(0.5, -0.25, 0.75));
  values.insert(P(1), Point(Vector3(1.3, -1.7, 2.8)));
  values.insert(V(1), Vector3(0.25, 0.1, 0.5));
  return values;
}

// Exact D=1 QCQP vectors of all AugmentedPoint and Vector3 entries.
Values lift(const Values& typed) {
  Values lifted;
  for (const auto& [key, point] : typed.extract<Point>()) {
    insertQcqpValue(key, point, lifted);
  }
  for (const auto& [key, velocity] : typed.extract<Vector3>()) {
    insertQcqpValue(key, velocity, lifted);
  }
  return lifted;
}

// Verifies that the motion cost is exact on the canonical lift and that only
// the pose keys add lift constraints.
TEST(WnoaAugmentedPointQcqp, MotionCostMatchesNonlinear) {
  const Values values = intervalValues();
  NonlinearFactorGraph graph;
  graph.emplace_shared<WnoaMotionFactor<Point>>(P(0), V(0), P(1), V(1), kDt,
                                                kPsd);
  const QcqpProblem problem(graph);

  const Values lifted = lift(values);
  CHECK(graph.error(values) > 1.0);
  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(lifted),
                       1e-9);
  EXPECT_LONGS_EQUAL(1, problem.costs().size());
  EXPECT_LONGS_EQUAL(4, problem.eConstraints().size());
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-12);
}

// Verifies the coordinate offset: plain Point3 poses have no lifted rows.
TEST(WnoaAugmentedPointQcqp, MotionCostWithPlainPoint3) {
  Values values;
  values.insert(P(0), Point3(1.0, -2.0, 3.0));
  values.insert(V(0), Vector3(0.5, -0.25, 0.75));
  values.insert(P(1), Point3(1.3, -1.7, 2.8));
  values.insert(V(1), Vector3(0.25, 0.1, 0.5));
  NonlinearFactorGraph graph;
  graph.emplace_shared<WnoaMotionFactor<Point3>>(P(0), V(0), P(1), V(1), kDt,
                                                 kPsd);
  const QcqpProblem problem(graph);

  Values lifted;
  for (const auto& [key, vector] : values.extract<Vector3>()) {
    insertQcqpValue(key, vector, lifted);
  }
  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(lifted),
                       1e-9);
  EXPECT_LONGS_EQUAL(0, problem.eConstraints().size());
}

// Verifies that an anisotropic WNOA power spectral density, which whitens each
// axis differently, keeps the motion cost exact.
TEST(WnoaAugmentedPointQcqp, AnisotropicPsdMatchesNonlinear) {
  const Vector3 psd(0.05, 0.4, 1.5);
  Values values = intervalValues();
  NonlinearFactorGraph graph;
  graph.emplace_shared<WnoaMotionFactor<Point>>(P(0), V(0), P(1), V(1), kDt,
                                                psd);
  const QcqpProblem problem(graph);

  const Values lifted = lift(values);
  CHECK(graph.error(values) > 1.0);
  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(lifted),
                       1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-12);
}

// Verifies that unsupported configurations fail loudly rather than
// approximating.
TEST(WnoaAugmentedPointQcqp, RejectsUnsupportedConfigurations) {
  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  QcqpVariableConstraints variableConstraints;

  WnoaMotionFactor<Point> motion(P(0), V(0), P(1), V(1), kDt, kPsd);
  CHECK_EXCEPTION(motion.qcqpFactors(costs, constraints, variableConstraints, 2),
                  std::invalid_argument);
  // Interpolated factors have no QCQP conversion of their own.
  const StateData left(P(0), V(0), 0.0), right(P(1), V(1), kDt),
      middle(P(2), V(2), 0.5 * kDt);
  WnoaInterpFactor<Point> interpolated(
      std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
          middle.pose, Vector3(2.0, 0.5, -1.0), 2.5,
          noiseModel::Isotropic::Sigma(1, 0.1)),
      {left, right}, {middle}, kPsd, true);
  CHECK_EXCEPTION(
      interpolated.qcqpFactors(costs, constraints, variableConstraints),
      std::runtime_error);
  WnoaMotionFactor<Pose2> pose2Motion(Key(0), Key(1), Key(2), Key(3), kDt,
                                      Vector3::Ones());
  CHECK_EXCEPTION(
      pose2Motion.qcqpFactors(costs, constraints, variableConstraints),
      std::runtime_error);
}

}  // namespace qcqp_fixture
/* ************************************************************************* */
namespace full_gp_fixture {

using Point = AugmentedPoint<3>;

const Vector3 kPsd(0.2, 0.3, 0.25);
const std::array<Vector3, 4> kLandmarks = {
    Vector3(0.0, 0.0, 0.0), Vector3(3.0, 0.0, 0.0), Vector3(0.0, 3.0, 0.0),
    Vector3(0.0, 0.0, 3.0)};

// Knots at t = 0, 1, 2, with three measurement times in the first interval and
// one in the second.
const std::set<StateData> kKnots = {StateData(P(0), V(0), 0.0),
                                    StateData(P(1), V(1), 1.0),
                                    StateData(P(2), V(2), 2.0)};
const std::set<StateData> kMeasurementStates = {
    StateData(P(10), V(10), 0.25), StateData(P(11), V(11), 0.5),
    StateData(P(12), V(12), 0.75), StateData(P(13), V(13), 1.4)};

Vector3 truePosition(double t) {
  return Vector3(0.6 + 0.35 * t, -0.2 + 0.1 * t * t, 0.7 - 0.15 * t);
}

// Every distinct state: the knots and the measurement-time states.
std::set<StateData> allStates() {
  std::set<StateData> states = kKnots;
  states.insert(kMeasurementStates.begin(), kMeasurementStates.end());
  return states;
}

// Squared ranges to all landmarks from the first knot and from each
// measurement-time state, each on the state's own key.
NonlinearFactorGraph measurementGraph() {
  NonlinearFactorGraph graph;
  auto addRanges = [&graph](const StateData& state) {
    for (size_t i = 0; i < kLandmarks.size(); ++i) {
      const double noise = (i % 2 == 0) ? 0.01 : -0.01;
      graph.emplace_shared<KnownLandmarkSquaredRangeFactor<Point>>(
          state.pose, kLandmarks[i],
          (truePosition(state.time) - kLandmarks[i]).norm() + noise,
          noiseModel::Isotropic::Sigma(1, 0.05));
    }
  };
  addRanges(*kKnots.begin());
  for (const StateData& state : kMeasurementStates) addRanges(state);
  return graph;
}

// Knot values offset from the true trajectory.
Values knotValues() {
  Values values;
  for (const StateData& state : kKnots) {
    values.insert(state.pose,
                  Point(truePosition(state.time) + Vector3(0.1, -0.05, 0.08)));
    values.insert(state.velocity, Vector3(0.3, 0.2, -0.1));
  }
  return values;
}

// Exact D=1 QCQP vectors of the states' positions and velocities.
Values lift(const Values& typed) {
  Values lifted;
  for (const StateData& state : allStates()) {
    insertQcqpValue(state.pose, typed.at<Point>(state.pose), lifted);
    insertQcqpValue(state.velocity, typed.at<Vector3>(state.velocity), lifted);
  }
  return lifted;
}

template <class FACTOR>
size_t countFactors(const NonlinearFactorGraph& graph) {
  return std::count_if(graph.begin(), graph.end(), [](const auto& factor) {
    return std::dynamic_pointer_cast<FACTOR>(factor) != nullptr;
  });
}

// Verifies the two partitions of one measurement graph: interpolating the
// measurement-time states wraps their factors, while estimating every state
// keeps the factors on their own states and links consecutive states.
TEST(WnoaAugmentedPointFullGp, StatePartitions) {
  const NonlinearFactorGraph measurements = measurementGraph();
  const NonlinearFactorGraph interpolated = interpolateFactorGraph<3>(
      measurements, kKnots, kMeasurementStates, kPsd, true);
  EXPECT_LONGS_EQUAL(2, countFactors<WnoaMotionFactor<Point>>(interpolated));
  EXPECT_LONGS_EQUAL(16, countFactors<WnoaInterpFactor<Point>>(interpolated));
  EXPECT_LONGS_EQUAL(6, interpolated.keys().size());

  const NonlinearFactorGraph fullGp =
      interpolateFactorGraph<3>(measurements, allStates(), {}, kPsd);
  EXPECT_LONGS_EQUAL(6, countFactors<WnoaMotionFactor<Point>>(fullGp));
  EXPECT_LONGS_EQUAL(0, countFactors<WnoaInterpFactor<Point>>(fullGp));
  EXPECT_LONGS_EQUAL(
      20, countFactors<KnownLandmarkSquaredRangeFactor<Point>>(fullGp));
  EXPECT_LONGS_EQUAL(14, fullGp.keys().size());
}

// Verifies that the full Gaussian process objective, with the measurement-time
// states at their WNOA conditional mean, equals the fixed-noise interpolated
// objective, and that the motion chain then equals the knot prior, so the
// interval prior is not counted twice.
TEST(WnoaAugmentedPointFullGp, ConditionalMeanMatchesFixedNoiseInterpolation) {
  const NonlinearFactorGraph measurements = measurementGraph();
  const NonlinearFactorGraph interpolated = interpolateFactorGraph<3>(
      measurements, kKnots, kMeasurementStates, kPsd, true);
  const NonlinearFactorGraph fullGp =
      interpolateFactorGraph<3>(measurements, allStates(), {}, kPsd);
  const Values knots = knotValues();
  const Values conditionalMean = updateInterpValues<3>(
      interpolated, knots, kKnots, kMeasurementStates, kPsd);

  const double interpolatedError = interpolated.error(knots);
  CHECK(interpolatedError > 1.0);
  EXPECT_DOUBLES_EQUAL(interpolatedError, fullGp.error(conditionalMean),
                       1e-9 * interpolatedError);

  const NonlinearFactorGraph knotPrior =
      interpolateFactorGraph<3>(NonlinearFactorGraph(), kKnots, {}, kPsd);
  const NonlinearFactorGraph chainPrior =
      interpolateFactorGraph<3>(NonlinearFactorGraph(), allStates(), {}, kPsd);
  CHECK(knotPrior.error(knots) > 1e-3);
  EXPECT_DOUBLES_EQUAL(knotPrior.error(knots),
                       chainPrior.error(conditionalMean), 1e-9);
}

// Verifies that the full Gaussian process graph converts to a QCQP with the
// same objective and only single-key constraints: the two lift equalities of
// each AugmentedPoint.
TEST(WnoaAugmentedPointFullGp, QcqpMatchesNonlinearWithUnaryConstraints) {
  const NonlinearFactorGraph fullGp =
      interpolateFactorGraph<3>(measurementGraph(), allStates(), {}, kPsd);
  const QcqpProblem problem(fullGp);
  EXPECT_LONGS_EQUAL(2 * allStates().size(), problem.eConstraints().size());
  for (const auto& constraint : problem.eConstraints()) {
    EXPECT_LONGS_EQUAL(1, constraint->size());
  }
  EXPECT_LONGS_EQUAL(0, problem.iConstraints().size());

  Values values;
  for (const StateData& state : allStates()) {
    values.insert(state.pose, Point(truePosition(state.time) +
                                    Vector3(0.05 * state.time, -0.04, 0.03)));
    values.insert(state.velocity, Vector3(0.3, 0.1 * state.time, -0.2));
  }
  const Values lifted = lift(values);
  const double error = fullGp.error(values);
  CHECK(error > 1.0);
  EXPECT_DOUBLES_EQUAL(error, problem.costs().error(lifted), 1e-9 * error);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-12);
}

// Verifies that the full Gaussian process QCQP, solved locally with an
// augmented Lagrangian and h = 1 on every point, reproduces the
// Levenberg-Marquardt solution of the nonlinear full Gaussian process problem.
TEST(WnoaAugmentedPointFullGp, AugmentedLagrangianMatchesLevenbergMarquardt) {
  const NonlinearFactorGraph measurements = measurementGraph();
  const NonlinearFactorGraph interpolated = interpolateFactorGraph<3>(
      measurements, kKnots, kMeasurementStates, kPsd, true);
  const NonlinearFactorGraph fullGp =
      interpolateFactorGraph<3>(measurements, allStates(), {}, kPsd);
  const Values initial = updateInterpValues<3>(
      interpolated, knotValues(), kKnots, kMeasurementStates, kPsd);
  const Values nonlinear =
      LevenbergMarquardtOptimizer(fullGp, initial).optimize();

  QcqpProblem problem(fullGp);
  Matrix homogeneous = Matrix::Zero(1, 5);
  homogeneous(0, 0) = 1.0;
  for (const StateData& state : allStates()) {
    problem.addConstraint(LinearConstraint::Equal(JacobianFactor(
        state.pose, homogeneous, Vector::Ones(1), noiseModel::Unit::Create(1))));
  }
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->updatePolicy = AugmentedLagrangianUpdatePolicy::Aggressive;
  params->maxIterations = 200;
  params->absoluteViolationTolerance = 1e-9;
  params->relativeViolationTolerance = 1e-9;
  params->relativeCostTolerance = 1e-12;
  params->absoluteStationarityTolerance = 1e-8;
  const Values solved =
      AugmentedLagrangianOptimizer(problem, lift(initial), params).optimize();

  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(solved), 1e-6);
  EXPECT_DOUBLES_EQUAL(fullGp.error(nonlinear), problem.costs().error(solved),
                       1e-6);
  for (const StateData& state : kKnots) {
    EXPECT(assert_equal(nonlinear.at<Point>(state.pose),
                        fromQcqpValue<Point>(solved.at<Matrix>(state.pose)),
                        1e-4));
    const Vector3 velocity = solved.at<Matrix>(state.velocity).col(0);
    EXPECT(assert_equal(nonlinear.at<Vector3>(state.velocity), velocity, 1e-4));
  }
}

}  // namespace full_gp_fixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
