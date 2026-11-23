/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testWnoaAugmentedPoint.cpp
 * @brief Tests WNOA motion factors and Gaussian process interpolation with
 * AugmentedPoint positions.
 * @author Oluwatodimu Afolayan
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaFactor.h>
#include <gtsam/nonlinear/WnoaInterpFactor.h>
#include <gtsam/nonlinear/WnoaInterpolator.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam_unstable/geometry/AugmentedPoint.h>
#include <gtsam_unstable/slam/KnownLandmarkSquaredRangeFactor.h>

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using symbol_shorthand::P;
using symbol_shorthand::V;

/* ************************************************************************* */
namespace wnoa_fixture {

using Point = AugmentedPoint<3>;

// Anisotropic power spectral density, so that each axis is weighted
// differently.
const Vector3 kPsd(0.2, 0.5, 1.5);

// Knots of one interval, deliberately not starting at t = 0.
const double kT0 = 1.0, kT1 = 1.4, kDt = kT1 - kT0;
const StateData kLeft(P(0), V(0), kT0), kRight(P(1), V(1), kT1);
const Vector3 kP0(1.0, -2.0, 3.0), kV0(0.5, -0.25, 0.75);
const Vector3 kP1(1.3, -1.7, 2.8), kV1(0.25, 0.1, 0.5);

// Knot values with positions stored as AugmentedPoint<3>.
Values knotValues(const Vector3& p0 = kP0, const Vector3& v0 = kV0,
                  const Vector3& p1 = kP1, const Vector3& v1 = kV1) {
  Values values;
  values.insert(kLeft.pose, Point(p0));
  values.insert(kLeft.velocity, v0);
  values.insert(kRight.pose, Point(p1));
  values.insert(kRight.velocity, v1);
  return values;
}

// Discrete WNOA covariance over [position; velocity] for a step dt, with the
// axes of each block ordered as in the factor residual.
Matrix6 wnoaCovariance(double dt, const Vector3& psd) {
  Matrix6 Q = Matrix6::Zero();
  for (int i = 0; i < 3; ++i) {
    Q(i, i) = psd(i) * dt * dt * dt / 3.0;
    Q(i, i + 3) = Q(i + 3, i) = psd(i) * dt * dt / 2.0;
    Q(i + 3, i + 3) = psd(i) * dt;
  }
  return Q;
}

// WNOA conditional mean of a vector space: the cubic Hermite interpolant of
// the knot positions and velocities, evaluated at offset s from the left knot.
std::pair<Vector3, Vector3> conditionalMean(double s, const Vector3& p0,
                                            const Vector3& v0,
                                            const Vector3& p1,
                                            const Vector3& v1) {
  const double u = s / kDt, u2 = u * u, u3 = u2 * u;
  const Vector3 position = (2 * u3 - 3 * u2 + 1) * p0 +
                           (u3 - 2 * u2 + u) * kDt * v0 +
                           (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * kDt * v1;
  const Vector3 velocity = (6 * u2 - 6 * u) / kDt * p0 +
                           (3 * u2 - 4 * u + 1) * v0 +
                           (-6 * u2 + 6 * u) / kDt * p1 +
                           (3 * u2 - 2 * u) * v1;
  return {position, velocity};
}

// Squared-range factor on an interpolated position key.
std::shared_ptr<KnownLandmarkSquaredRangeFactor<Point>> rangeFactor(
    Key key, const Vector3& landmark, double range, double sigma) {
  return std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
      key, landmark, range, noiseModel::Isotropic::Sigma(1, sigma));
}

// Interpolation of the fixture interval at time t.
Interpolator<Point>::PoseVel interpolate(const Interpolator<Point>& interpolator,
                                         double t) {
  return interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point>(Point(kP0), kV0, kT0),
      TimestampedPoseVelocity<Point>(Point(kP1), kV1, kT1), t);
}

// Calls the explicitly instantiated Interpolator<AugmentedPoint<N>> at an
// interior time and returns the error against the WNOA conditional mean.
template <int N>
double instantiationError() {
  using PointN = AugmentedPoint<N>;
  using VectorN = Eigen::Matrix<double, N, 1>;
  const VectorN p0 = kP0.head<N>(), v0 = kV0.head<N>();
  const VectorN p1 = kP1.head<N>(), v1 = kV1.head<N>();
  const Interpolator<PointN> interpolator(kPsd.head<N>());
  const auto [pose, vel] = interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<PointN>(PointN(p0), v0, kT0),
      TimestampedPoseVelocity<PointN>(PointN(p1), v1, kT1), kT0 + 0.15);
  Vector3 pad0 = Vector3::Zero(), pad1 = Vector3::Zero();
  Vector3 padv0 = Vector3::Zero(), padv1 = Vector3::Zero();
  pad0.head<N>() = p0;
  pad1.head<N>() = p1;
  padv0.head<N>() = v0;
  padv1.head<N>() = v1;
  const auto [position, velocity] =
      conditionalMean(0.15, pad0, padv0, pad1, padv1);
  return (pose.vector() - position.head<N>()).norm() +
         (vel - velocity.head<N>()).norm();
}

// Verifies that the explicit Interpolator<AugmentedPoint<2>> and
// Interpolator<AugmentedPoint<3>> instantiations link and interpolate.
TEST(WnoaAugmentedPoint, InterpolatorInstantiations) {
  EXPECT_DOUBLES_EQUAL(0.0, instantiationError<2>(), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, instantiationError<3>(), 1e-12);
}

// Verifies that a motion factor built from StateData uses its keys and time
// step, and that a constant-velocity trajectory has zero residual and error.
TEST(WnoaAugmentedPoint, MotionFromStateData) {
  const WnoaMotionFactor<Point> factor(kLeft, kRight, kPsd);
  EXPECT(factor.keys() ==
         KeyVector({kLeft.pose, kLeft.velocity, kRight.pose, kRight.velocity}));

  const Values values = knotValues(kP0, kV0, kP0 + kDt * kV0, kV0);
  EXPECT(assert_equal(Vector(Vector6::Zero()), factor.unwhitenedError(values),
                      1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-12);

  // The same factor works for two-dimensional positions.
  const StateData left2(P(5), V(5), kT0), right2(P(6), V(6), kT1);
  const WnoaMotionFactor<AugmentedPoint<2>> factor2(left2, right2,
                                                    Vector2(0.2, 0.5));
  const Vector2 p(1.0, -2.0), v(0.5, -0.25);
  EXPECT(assert_equal(
      Vector(Vector4::Zero()),
      factor2.evaluateError(AugmentedPoint<2>(p), v,
                            AugmentedPoint<2>(p + kDt * v), v),
      1e-12));
}

// Verifies the residual [p1 - p0 - dt v0; v1 - v0] for perturbed positions and
// velocities, and the error 1/2 r' Q^-1 r with the WNOA process covariance Q.
TEST(WnoaAugmentedPoint, MotionPerturbedResidual) {
  const WnoaMotionFactor<Point> factor(kLeft, kRight, kPsd);
  const Vector3 dp(0.03, -0.02, 0.05), dv(-0.1, 0.2, 0.05);
  const Values values = knotValues(kP0, kV0, kP0 + kDt * kV0 + dp, kV0 + dv);

  Vector6 expected;
  expected << dp, dv;
  EXPECT(assert_equal(Vector(expected), factor.unwhitenedError(values), 1e-12));
  const double expectedError =
      0.5 * expected.dot(wnoaCovariance(kDt, kPsd).ldlt().solve(expected));
  EXPECT_DOUBLES_EQUAL(expectedError, factor.error(values),
                       1e-9 * expectedError);
}

// Verifies the analytical motion Jacobians against numerical derivatives.
TEST(WnoaAugmentedPoint, MotionJacobians) {
  const WnoaMotionFactor<Point> factor(kLeft, kRight, kPsd);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, knotValues(), 1e-7, 1e-5);
}

// Verifies that AugmentedPoint<3> and Point3 positions give the same motion
// residual, error and Jacobians.
TEST(WnoaAugmentedPoint, MotionMatchesPoint3) {
  const WnoaMotionFactor<Point> augmented(kLeft, kRight, kPsd);
  const WnoaMotionFactor<Point3> plain(kLeft, kRight, kPsd);
  std::vector<Matrix> Ha(4), Hp(4);
  const Vector ea =
      augmented.evaluateError(Point(kP0), kV0, Point(kP1), kV1, Ha[0], Ha[1],
                              Ha[2], Ha[3]);
  const Vector ep =
      plain.evaluateError(kP0, kV0, kP1, kV1, Hp[0], Hp[1], Hp[2], Hp[3]);
  EXPECT(assert_equal(ep, ea, 1e-12));
  for (size_t i = 0; i < 4; ++i) EXPECT(assert_equal(Hp[i], Ha[i], 1e-12));

  Values plainValues;
  plainValues.insert(kLeft.pose, kP0);
  plainValues.insert(kLeft.velocity, kV0);
  plainValues.insert(kRight.pose, kP1);
  plainValues.insert(kRight.velocity, kV1);
  EXPECT_DOUBLES_EQUAL(plain.error(plainValues), augmented.error(knotValues()),
                       1e-12);
}

}  // namespace wnoa_fixture
/* ************************************************************************* */
namespace interpolation_fixture {

using namespace wnoa_fixture;

// Verifies that interpolated positions and velocities equal the WNOA
// conditional mean (cubic Hermite interpolation) at interior times.
TEST(WnoaAugmentedPoint, InterpolationMatchesConditionalMean) {
  const Interpolator<Point> interpolator(kPsd);
  for (double s : {0.05, 0.1, 0.2, 0.3, 0.37}) {
    const auto [pose, vel] = interpolate(interpolator, kT0 + s);
    const auto [position, velocity] = conditionalMean(s, kP0, kV0, kP1, kV1);
    EXPECT(assert_equal(position, pose.vector(), 1e-12));
    EXPECT(assert_equal(velocity, vel, 1e-12));
  }
}

// Verifies the analytical interpolation Jacobians with respect to both knot
// positions and velocities against numerical derivatives.
TEST(WnoaAugmentedPoint, InterpolationJacobians) {
  const Interpolator<Point> interpolator(kPsd);
  const double tau = kT0 + 0.13;
  std::vector<Matrix> H(8);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point>(Point(kP0), kV0, kT0),
      TimestampedPoseVelocity<Point>(Point(kP1), kV1, kT1), tau, &H);

  // Interpolated [position; velocity] as a function of the knot states.
  auto state = [&](const Point& p0, const Vector3& v0, const Point& p1,
                   const Vector3& v1) -> Vector6 {
    const auto [pose, vel] = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point>(p0, v0, kT0),
        TimestampedPoseVelocity<Point>(p1, v1, kT1), tau);
    Vector6 result;
    result << pose.vector(), vel;
    return result;
  };
  const Point p0(kP0), p1(kP1);
  const std::array<Matrix, 4> numerical = {
      numericalDerivative41<Vector6, Point, Vector3, Point, Vector3>(
          state, p0, kV0, p1, kV1),
      numericalDerivative42<Vector6, Point, Vector3, Point, Vector3>(
          state, p0, kV0, p1, kV1),
      numericalDerivative43<Vector6, Point, Vector3, Point, Vector3>(
          state, p0, kV0, p1, kV1),
      numericalDerivative44<Vector6, Point, Vector3, Point, Vector3>(
          state, p0, kV0, p1, kV1)};
  // H holds the four position blocks, then the four velocity blocks.
  for (size_t i = 0; i < 4; ++i) {
    EXPECT(assert_equal(Matrix(numerical[i].topRows<3>()), H[i], 1e-7));
    EXPECT(assert_equal(Matrix(numerical[i].bottomRows<3>()), H[i + 4], 1e-7));
  }
}

// Verifies that interpolation returns the knots exactly at the interval
// endpoints and is continuous next to them.
TEST(WnoaAugmentedPoint, InterpolationAtEndpoints) {
  const Interpolator<Point> interpolator(kPsd);
  const auto left = interpolate(interpolator, kT0);
  const auto right = interpolate(interpolator, kT1);
  EXPECT(assert_equal(kP0, left.pose.vector(), 1e-12));
  EXPECT(assert_equal(kV0, left.vel, 1e-12));
  EXPECT(assert_equal(kP1, right.pose.vector(), 1e-12));
  EXPECT(assert_equal(kV1, right.vel, 1e-12));

  const auto nearLeft = interpolate(interpolator, kT0 + 1e-9);
  const auto nearRight = interpolate(interpolator, kT1 - 1e-9);
  EXPECT(assert_equal(kP0, nearLeft.pose.vector(), 1e-8));
  EXPECT(assert_equal(kV0, nearLeft.vel, 1e-6));
  EXPECT(assert_equal(kP1, nearRight.pose.vector(), 1e-8));
  EXPECT(assert_equal(kV1, nearRight.vel, 1e-6));
}

// Verifies the existing forward WNOA prediction from a single state: the
// prior mean keeps the velocity constant.
TEST(WnoaAugmentedPoint, ForwardPrediction) {
  const Interpolator<Point> interpolator(kPsd);
  const auto [pose, vel] = interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point>(Point(kP1), kV1, kT1), std::nullopt,
      kT1 + 0.3);
  EXPECT(assert_equal(Vector3(kP1 + 0.3 * kV1), pose.vector(), 1e-12));
  EXPECT(assert_equal(kV1, vel, 1e-12));
}

// Verifies that AugmentedPoint<3> and Point3 interpolation give the same
// states and Jacobians.
TEST(WnoaAugmentedPoint, InterpolationMatchesPoint3) {
  const Interpolator<Point> augmented(kPsd);
  const Interpolator<Point3> plain(kPsd);
  const double tau = kT0 + 0.22;
  std::vector<Matrix> Ha(8), Hp(8);
  const auto a = augmented.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point>(Point(kP0), kV0, kT0),
      TimestampedPoseVelocity<Point>(Point(kP1), kV1, kT1), tau, &Ha);
  const auto p = plain.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point3>(kP0, kV0, kT0),
      TimestampedPoseVelocity<Point3>(kP1, kV1, kT1), tau, &Hp);
  EXPECT(assert_equal(Vector3(p.pose), a.pose.vector(), 1e-12));
  EXPECT(assert_equal(p.vel, a.vel, 1e-12));
  for (size_t i = 0; i < 8; ++i) EXPECT(assert_equal(Hp[i], Ha[i], 1e-12));
}

}  // namespace interpolation_fixture
/* ************************************************************************* */
namespace interpolated_range_fixture {

using namespace wnoa_fixture;

const double kTau = kT0 + 0.13, kSigma = 0.1, kRange = 2.5;
const Vector3 kLandmark(2.0, 0.5, -1.0);
const StateData kMiddle(P(2), V(2), kTau);

// Interpolated squared-range factor between the fixture knots.
WnoaInterpFactor<Point> interpolatedRange(bool fixedNoise) {
  return WnoaInterpFactor<Point>(
      rangeFactor(kMiddle.pose, kLandmark, kRange, kSigma), {kLeft, kRight},
      {kMiddle}, kPsd, fixedNoise);
}

// Squared-range residual at the conditional-mean position.
double expectedResidual() {
  const Vector3 q = conditionalMean(kTau - kT0, kP0, kV0, kP1, kV1).first;
  return (q - kLandmark).squaredNorm() - kRange * kRange;
}

// Position variance of the WNOA conditional distribution at offset s, per
// axis: Q(s) - Q(s) Phi' Q(dt)^-1 Phi Q(s), with Phi the transition from s to
// the right knot.
Vector3 conditionalPositionVariance(double s) {
  Vector3 variance;
  for (int i = 0; i < 3; ++i) {
    auto Q = [&](double h) -> Matrix2 {
      return kPsd(i) * Matrix2{{h * h * h / 3, h * h / 2}, {h * h / 2, h}};
    };
    const Matrix2 Phi{{1.0, kDt - s}, {0.0, 1.0}};
    const Matrix2 P =
        Q(s) - Q(s) * Phi.transpose() * Q(kDt).inverse() * Phi * Q(s);
    variance(i) = P(0, 0);
  }
  return variance;
}

// Verifies the interpolated squared-range residual and error, and that the
// factor depends on the positions and velocities of both neighboring knots.
TEST(WnoaAugmentedPoint, InterpolatedSquaredRange) {
  const WnoaInterpFactor<Point> factor = interpolatedRange(true);
  const KeyVector& keys = factor.keys();
  EXPECT(KeySet(keys.begin(), keys.end()) ==
         KeySet({kLeft.pose, kLeft.velocity, kRight.pose, kRight.velocity}));

  const Values values = knotValues();
  const double residual = expectedResidual();
  EXPECT(assert_equal(Vector1(residual), factor.unwhitenedError(values), 1e-12));
  EXPECT_DOUBLES_EQUAL(0.5 * residual * residual / (kSigma * kSigma),
                       factor.error(values), 1e-9);

  const auto jacobian =
      std::dynamic_pointer_cast<JacobianFactor>(factor.linearize(values));
  CHECK(jacobian);
  for (auto it = jacobian->begin(); it != jacobian->end(); ++it) {
    EXPECT(jacobian->getA(it).norm() > 1e-3);
  }
}

// Verifies the interpolated squared-range Jacobians against numerical
// derivatives. The interpolation-noise mode linearizes with its noise model
// frozen at the linearization point, so its unwhitened Jacobians must equal
// those of the fixed-noise mode.
TEST(WnoaAugmentedPoint, InterpolatedSquaredRangeJacobians) {
  Values values = knotValues();
  const WnoaInterpFactor<Point> fixed = interpolatedRange(true);
  EXPECT_CORRECT_FACTOR_JACOBIANS(fixed, values, 1e-6, 1e-5);

  const WnoaInterpFactor<Point> inflated = interpolatedRange(false);
  const double sigma =
      std::sqrt(inflated.noiseModel(values)->covariance()(0, 0));
  const auto fixedJacobian =
      std::dynamic_pointer_cast<JacobianFactor>(fixed.linearize(values));
  const auto inflatedJacobian =
      std::dynamic_pointer_cast<JacobianFactor>(inflated.linearize(values));
  CHECK(fixedJacobian && inflatedJacobian);
  for (Key key : fixed.keys()) {
    EXPECT(assert_equal(
        Matrix(kSigma * fixedJacobian->getA(fixedJacobian->find(key))),
        Matrix(sigma * inflatedJacobian->getA(inflatedJacobian->find(key))),
        1e-9));
  }
}

// Verifies both noise modes: the fixed mode keeps the measurement noise, and
// the interpolated mode adds the conditional position covariance propagated
// through the squared-range Jacobian 2 (q - l)'.
TEST(WnoaAugmentedPoint, InterpolationNoiseModes) {
  Values values = knotValues();
  const double residual = expectedResidual();

  const WnoaInterpFactor<Point> fixed = interpolatedRange(true);
  EXPECT(assert_equal(Matrix(Matrix1::Constant(kSigma * kSigma)),
                      fixed.noiseModel(values)->covariance(), 1e-12));

  const WnoaInterpFactor<Point> inflated = interpolatedRange(false);
  const Vector3 q = conditionalMean(kTau - kT0, kP0, kV0, kP1, kV1).first;
  const Vector3 J = 2 * (q - kLandmark);
  const double variance =
      kSigma * kSigma +
      J.cwiseProduct(J).dot(conditionalPositionVariance(kTau - kT0));
  EXPECT(assert_equal(Matrix(Matrix1::Constant(variance)),
                      inflated.noiseModel(values)->covariance(), 1e-9));
  EXPECT(assert_equal(Vector1(residual), inflated.unwhitenedError(values),
                      1e-12));
  EXPECT_DOUBLES_EQUAL(0.5 * residual * residual / variance,
                       inflated.error(values), 1e-9);
  EXPECT(inflated.error(values) < fixed.error(values));
}

// Verifies that AugmentedPoint<3> and Point3 interpolated squared-range
// factors give the same error in both noise modes.
TEST(WnoaAugmentedPoint, InterpolatedSquaredRangeMatchesPoint3) {
  Values plainValues;
  plainValues.insert(kLeft.pose, kP0);
  plainValues.insert(kLeft.velocity, kV0);
  plainValues.insert(kRight.pose, kP1);
  plainValues.insert(kRight.velocity, kV1);
  for (bool fixedNoise : {true, false}) {
    const WnoaInterpFactor<Point3> plain(
        std::make_shared<KnownLandmarkSquaredRangeFactor<Vector3>>(
            kMiddle.pose, kLandmark, kRange,
            noiseModel::Isotropic::Sigma(1, kSigma)),
        {kLeft, kRight}, {kMiddle}, kPsd, fixedNoise);
    EXPECT_DOUBLES_EQUAL(plain.error(plainValues),
                         interpolatedRange(fixedNoise).error(knotValues()),
                         1e-9);
  }
}

// Verifies that interpolation times outside the estimated knots are rejected
// rather than extrapolated.
TEST(WnoaAugmentedPoint, RejectsTimesOutsideKnots) {
  auto factorAt = [](double t) {
    const StateData outside(P(3), V(3), t);
    return WnoaInterpFactor<Point>(
        rangeFactor(outside.pose, kLandmark, kRange, kSigma), {kLeft, kRight},
        {outside}, kPsd, true);
  };
  CHECK_EXCEPTION(factorAt(kT0 - 0.1), std::runtime_error);
  CHECK_EXCEPTION(factorAt(kT1 + 0.1), std::runtime_error);
}

}  // namespace interpolated_range_fixture
/* ************************************************************************* */
namespace optimization_fixture {

using Point = AugmentedPoint<3>;

// Verifies that range-only measurements at intermediate times, with WNOA
// motion priors between three knots, recover a constant-velocity trajectory
// from perturbed initial values.
TEST(WnoaAugmentedPoint, RangeOnlyOptimization) {
  const Vector3 p0(0.6, -0.2, 0.7), velocity(0.35, 0.1, -0.15);
  const Vector3 psd = Vector3::Constant(0.2);
  const std::set<StateData> knots = {StateData(P(0), V(0), 0.0),
                                     StateData(P(1), V(1), 1.0),
                                     StateData(P(2), V(2), 2.0)};
  const std::array<Vector3, 4> landmarks = {
      Vector3(0.0, 0.0, 0.0), Vector3(3.0, 0.0, 0.0), Vector3(0.0, 3.0, 0.0),
      Vector3(0.0, 0.0, 3.0)};
  const auto noise = noiseModel::Isotropic::Sigma(1, 0.02);

  NonlinearFactorGraph graph;
  for (auto it = knots.begin(); std::next(it) != knots.end(); ++it) {
    graph.emplace_shared<WnoaMotionFactor<Point>>(*it, *std::next(it), psd);
  }
  // Ranges at the first knot anchor the trajectory.
  for (const Vector3& landmark : landmarks) {
    graph.emplace_shared<KnownLandmarkSquaredRangeFactor<Point>>(
        P(0), landmark, (p0 - landmark).norm(), noise);
  }
  // Ranges between the knots act on interpolated positions.
  const std::array<double, 4> times = {0.25, 0.75, 1.25, 1.75};
  for (size_t j = 0; j < times.size(); ++j) {
    const StateData at(P(10 + j), V(10 + j), times[j]);
    for (const Vector3& landmark : landmarks) {
      auto inner = std::make_shared<KnownLandmarkSquaredRangeFactor<Point>>(
          at.pose, landmark, (p0 + times[j] * velocity - landmark).norm(),
          noise);
      graph.emplace_shared<WnoaInterpFactor<Point>>(
          inner, knots, std::set<StateData>{at}, psd, true);
    }
  }

  Values initial;
  for (size_t k = 0; k < 3; ++k) {
    initial.insert(P(k), Point(p0 + double(k) * velocity +
                               Vector3(0.12, -0.08, 0.1)));
    initial.insert(V(k), Vector3(velocity + Vector3(-0.08, 0.06, 0.1)));
  }
  const double initialError = graph.error(initial);
  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

  CHECK(initialError > 1.0);
  EXPECT(graph.error(result) < 1e-8);
  EXPECT_LONGS_EQUAL(6, result.size());  // No states at measurement times.
  for (size_t k = 0; k < 3; ++k) {
    EXPECT(assert_equal(Vector3(p0 + double(k) * velocity),
                        result.at<Point>(P(k)).vector(), 1e-4));
    EXPECT(assert_equal(velocity, result.at<Vector3>(V(k)), 1e-4));
  }
}

}  // namespace optimization_fixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
