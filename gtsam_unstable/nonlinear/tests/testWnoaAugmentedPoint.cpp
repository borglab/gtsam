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
namespace qcqp_fixture {

using Point = AugmentedPoint<3>;

const Vector3 kPsd = Vector3::Constant(0.2);
const double kDt = 0.2, kTau = 0.08;
const Key kInterp = P(2);

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

// WNOA interpolation of the interval's knots at kTau.
Interpolator<Point>::PoseVel interpolate(const Values& values,
                                         const Vector3& psd = kPsd) {
  return Interpolator<Point>(psd).interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point>(values.at<Point>(P(0)),
                                     values.at<Vector3>(V(0)), 0.0),
      TimestampedPoseVelocity<Point>(values.at<Point>(P(1)),
                                     values.at<Vector3>(V(1)), kDt),
      kTau);
}

// Interpolation wrapper of a squared-range factor on the interpolated key.
std::shared_ptr<WnoaInterpFactor<Point>> rangeFactor(
    const Vector3& landmark, double range, size_t index = 2,
    bool fixedNoise = true, const Vector3& psd = kPsd) {
  auto inner = std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
      P(index), landmark, range, noiseModel::Isotropic::Sigma(1, 0.1));
  return std::make_shared<WnoaInterpFactor<Point>>(
      inner,
      std::set<StateData>{StateData(P(0), V(0), 0.0),
                          StateData(P(1), V(1), kDt)},
      std::set<StateData>{StateData(P(index), V(index), kTau)}, psd,
      fixedNoise);
}

// Unary linear cost on a velocity key, with QCQP support, for testing the
// interpolated-velocity coupling.
class VelocityPrior : public NoiseModelFactorN<Vector3> {
  Vector3 measured_;

 public:
  VelocityPrior(Key key, const Vector3& measured,
                const SharedNoiseModel& model)
      : NoiseModelFactorN<Vector3>(model, key), measured_(measured) {}

  Vector evaluateError(const Vector3& velocity,
                       OptionalMatrixType H) const override {
    if (H) *H = Matrix3::Identity();
    return velocity - measured_;
  }

  void qcqpFactors(NonlinearFactorGraph& costs, NonlinearEqualityConstraints&,
                   QcqpVariableConstraints& variableConstraints,
                   size_t) const override {
    Matrix B(3, 4);  // [A b] for the residual A*v - b.
    B << Matrix3::Identity(), measured_;
    const Matrix whitened = noiseModel_->Whiten(B);
    costs.push_back(std::make_shared<QpCost>(HessianFactor(
        KeyVector{key()}, SymmetricBlockMatrix(std::vector<DenseIndex>{3, 1},
                                               whitened.transpose() *
                                                   whitened))));
    variableConstraints.add(key(), traits<Vector3>::QcqpConstraints<1>);
  }
};

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

// Verifies the interpolated range: exact cost on the lifted interpolated point,
// its lift constraints, and the three linear coupling equations.
TEST(WnoaAugmentedPointQcqp, InterpolatedRangeMatchesNonlinear) {
  Values values = intervalValues();
  NonlinearFactorGraph graph;
  graph.push_back(rangeFactor(Vector3(2.0, 0.5, -1.0), 2.5));
  const QcqpProblem problem(graph);

  const Point interpolated = interpolate(values).pose;
  Values lifted = lift(values);
  insertQcqpValue(kInterp, interpolated, lifted);

  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(lifted),
                       1e-9);
  // Lift constraints of P0, P1 and the interpolated key, plus one coupling.
  EXPECT_LONGS_EQUAL(7, problem.eConstraints().size());
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-9);

  // A lifted point that is not the interpolation violates the coupling.
  Values moved = lift(values);
  insertQcqpValue(kInterp, interpolated + Vector3(0.1, 0.0, 0.0), moved);
  EXPECT(problem.eConstraints().violationNorm(moved) > 1e-3);
}

// Verifies that factors sharing an interpolated key add its coupling once.
TEST(WnoaAugmentedPointQcqp, SharedInterpolatedKeyCouplesOnce) {
  const std::array<Vector3, 3> landmarks = {
      Vector3(2.0, 0.5, -1.0), Vector3(-1.0, 3.0, 0.5), Vector3(0.0, 0.0, 4.0)};
  NonlinearFactorGraph shared, separate;
  for (size_t i = 0; i < landmarks.size(); ++i) {
    shared.push_back(rangeFactor(landmarks[i], 2.5));
    separate.push_back(rangeFactor(landmarks[i], 2.5, 2 + i));
  }
  const QcqpProblem sharedProblem(shared), separateProblem(separate);
  EXPECT_LONGS_EQUAL(3, sharedProblem.costs().size());
  EXPECT_LONGS_EQUAL(4 + 2 + 1, sharedProblem.eConstraints().size());
  EXPECT_LONGS_EQUAL(3, separateProblem.costs().size());
  EXPECT_LONGS_EQUAL(4 + 3 * 2 + 3, separateProblem.eConstraints().size());
}

// Verifies the interpolated-velocity coupling against the Interpolator.
TEST(WnoaAugmentedPointQcqp, InterpolatedVelocityKeyCoupling) {
  Values values = intervalValues();
  const Key velocityKey = V(2);
  auto inner = std::make_shared<VelocityPrior>(
      velocityKey, Vector3(0.3, 0.2, 0.1), noiseModel::Isotropic::Sigma(3, 0.1));
  NonlinearFactorGraph graph;
  graph.emplace_shared<WnoaInterpFactor<Point>>(
      inner,
      std::set<StateData>{StateData(P(0), V(0), 0.0),
                          StateData(P(1), V(1), kDt)},
      std::set<StateData>{StateData(P(2), velocityKey, kTau)}, kPsd, true);
  const QcqpProblem problem(graph);

  const Vector3 velocity = interpolate(values).vel;
  Values lifted = lift(values);
  insertQcqpValue(velocityKey, velocity, lifted);
  EXPECT_DOUBLES_EQUAL(0.5 * (velocity - Vector3(0.3, 0.2, 0.1)).squaredNorm() /
                           0.01,
                       problem.costs().error(lifted), 1e-9);
  EXPECT_LONGS_EQUAL(4 + 1, problem.eConstraints().size());
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-9);
  Values moved = lift(values);
  insertQcqpValue(velocityKey, Vector3(velocity + Vector3(0.0, 0.1, 0.0)),
                  moved);
  EXPECT(problem.eConstraints().violationNorm(moved) > 1e-3);
}

// Verifies that anisotropic WNOA power spectral densities, which make the
// whitened motion cost and the interpolation differ per axis, stay exact.
TEST(WnoaAugmentedPointQcqp, AnisotropicPsdMatchesNonlinear) {
  const Vector3 psd(0.05, 0.4, 1.5);
  Values values = intervalValues();
  NonlinearFactorGraph graph;
  graph.emplace_shared<WnoaMotionFactor<Point>>(P(0), V(0), P(1), V(1), kDt,
                                                psd);
  graph.push_back(rangeFactor(Vector3(2.0, 0.5, -1.0), 2.5, 2, true, psd));
  const QcqpProblem problem(graph);

  Values lifted = lift(values);
  insertQcqpValue(kInterp, interpolate(values, psd).pose, lifted);
  CHECK(graph.error(values) > 1.0);
  EXPECT_DOUBLES_EQUAL(graph.error(values), problem.costs().error(lifted),
                       1e-9);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-9);
}

// Verifies the sign gauge: negating a lifted knot column still satisfies its
// quadratic lift constraints but violates the linear coupling, which is why
// every AugmentedPoint needs the unary constraint h = 1.
TEST(WnoaAugmentedPointQcqp, NegativeHomogeneousCoordinateBreaksCoupling) {
  const Values values = intervalValues();
  NonlinearFactorGraph graph;
  graph.push_back(rangeFactor(Vector3(2.0, 0.5, -1.0), 2.5));
  const QcqpProblem problem(graph);

  Values lifted = lift(values);
  insertQcqpValue(kInterp, interpolate(values).pose, lifted);
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(lifted),
                       1e-9);

  // The negated column is s*[1, x, |x|^2] with s = -1.
  Values flipped = lifted;
  flipped.update(P(0), Matrix(-lifted.at<Matrix>(P(0))));
  EXPECT(problem.eConstraints().violationNorm(flipped) > 1e-3);
}

// Verifies that unsupported configurations fail loudly rather than
// approximating.
TEST(WnoaAugmentedPointQcqp, RejectsUnsupportedConfigurations) {
  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  QcqpVariableConstraints variableConstraints;

  const auto adaptiveNoise = rangeFactor(Vector3(2.0, 0.5, -1.0), 2.5, 2, false);
  CHECK_EXCEPTION(adaptiveNoise->qcqpFactors(costs, constraints,
                                             variableConstraints),
                  std::runtime_error);
  const auto range = rangeFactor(Vector3(2.0, 0.5, -1.0), 2.5);
  CHECK_EXCEPTION(range->qcqpFactors(costs, constraints, variableConstraints, 2),
                  std::invalid_argument);
  WnoaMotionFactor<Point> motion(P(0), V(0), P(1), V(1), kDt, kPsd);
  CHECK_EXCEPTION(motion.qcqpFactors(costs, constraints, variableConstraints, 2),
                  std::invalid_argument);
  WnoaMotionFactor<Pose2> pose2Motion(Key(0), Key(1), Key(2), Key(3), kDt,
                                      Vector3::Ones());
  CHECK_EXCEPTION(
      pose2Motion.qcqpFactors(costs, constraints, variableConstraints),
      std::runtime_error);
}

// Verifies that the QCQP solved locally with ALM, with h = 1 on every point,
// reproduces the Levenberg-Marquardt solution of the nonlinear range-only
// problem with shared interpolated keys.
TEST(WnoaAugmentedPointQcqp, RangeOnlyAugmentedLagrangianMatchesNonlinear) {
  const Vector3 p0(0.6, -0.2, 0.7), velocity(0.35, 0.1, -0.15);
  const std::set<StateData> knots = {StateData(P(0), V(0), 0.0),
                                     StateData(P(1), V(1), 1.0),
                                     StateData(P(2), V(2), 2.0)};
  const std::array<Vector3, 4> landmarks = {
      Vector3(0.0, 0.0, 0.0), Vector3(3.0, 0.0, 0.0), Vector3(0.0, 3.0, 0.0),
      Vector3(0.0, 0.0, 3.0)};
  const std::array<double, 4> times = {0.25, 0.75, 1.25, 1.75};

  NonlinearFactorGraph graph;
  for (size_t k = 0; k < 2; ++k) {
    graph.emplace_shared<WnoaMotionFactor<Point>>(
        StateData(P(k), V(k), double(k)),
        StateData(P(k + 1), V(k + 1), double(k + 1)), kPsd);
  }
  for (size_t j = 0; j < times.size(); ++j) {
    const StateData interp(P(10 + j), V(10 + j), times[j]);
    for (const Vector3& landmark : landmarks) {
      auto inner = std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
          interp.pose, landmark, (p0 + times[j] * velocity - landmark).norm(),
          noiseModel::Isotropic::Sigma(1, 0.05));
      graph.emplace_shared<WnoaInterpFactor<Point>>(
          inner, knots, std::set<StateData>{interp}, kPsd, true);
    }
  }

  Values initial;
  for (size_t k = 0; k < 3; ++k) {
    initial.insert(P(k), Point(p0 + double(k) * velocity +
                               Vector3(0.05, -0.04, 0.03)));
    initial.insert(V(k), Vector3(velocity + Vector3(-0.04, 0.03, 0.05)));
  }
  const Values nonlinear = LevenbergMarquardtOptimizer(graph, initial).optimize();

  // Lifted initial values: knots plus one interpolated point per time.
  Values liftedInitial = lift(initial);
  Interpolator<Point> interpolator(kPsd);
  QcqpProblem problem(graph);
  for (size_t j = 0; j < times.size(); ++j) {
    const size_t k = times[j] < 1.0 ? 0 : 1;
    const auto pv = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point>(initial.at<Point>(P(k)),
                                       initial.at<Vector3>(V(k)), double(k)),
        TimestampedPoseVelocity<Point>(initial.at<Point>(P(k + 1)),
                                       initial.at<Vector3>(V(k + 1)),
                                       double(k + 1)),
        times[j]);
    insertQcqpValue(P(10 + j), pv.pose, liftedInitial);
  }
  Matrix homogeneous = Matrix::Zero(1, 5);
  homogeneous(0, 0) = 1.0;
  for (const auto& [key, lifted] : liftedInitial.extract<Matrix>()) {
    if (lifted.rows() != 5) continue;
    problem.addConstraint(LinearConstraint::Equal(JacobianFactor(
        key, homogeneous, Vector::Ones(1), noiseModel::Unit::Create(1))));
  }
  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(liftedInitial),
                       1e-9);

  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->maxIterations = 200;
  params->absoluteViolationTolerance = 1e-9;
  params->relativeViolationTolerance = 1e-9;
  params->relativeCostTolerance = 1e-12;
  params->absoluteStationarityTolerance = 1e-8;
  const Values solved =
      AugmentedLagrangianOptimizer(problem, liftedInitial, params).optimize();

  EXPECT_DOUBLES_EQUAL(0.0, problem.eConstraints().violationNorm(solved), 1e-6);
  for (size_t k = 0; k < 3; ++k) {
    const Point recovered = fromQcqpValue<Point>(solved.at<Matrix>(P(k)));
    EXPECT(assert_equal(nonlinear.at<Point>(P(k)), recovered, 1e-4));
    const Vector3 recoveredVelocity = solved.at<Matrix>(V(k)).col(0);
    EXPECT(assert_equal(nonlinear.at<Vector3>(V(k)), recoveredVelocity, 1e-4));
  }
  EXPECT_DOUBLES_EQUAL(graph.error(nonlinear), problem.costs().error(solved),
                       1e-6);
}

}  // namespace qcqp_fixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
