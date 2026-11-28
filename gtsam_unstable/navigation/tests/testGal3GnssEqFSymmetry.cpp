/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGal3GnssEqFSymmetry.cpp
 * @brief Tests for the Galilean action, lift and EqF matrices.
 * @author Moses Adewolu
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/Gal3GnssEqF.h>

#include <algorithm>
#include <random>
#include <vector>

using namespace gtsam;
using namespace gtsam::galilean;

namespace {
const Vector3 kGravity(0, 0, -9.81);
const Vector10 kEarth = EarthInput(kGravity, Vector3(7e-5, 1e-5, -3e-5));
const IMUInput kImu(Vector3(0.3, -0.1, 0.4), Vector3(1.0, -0.5, 9.0));
const double kDt = 0.005;

Vector10 randomVector10(std::mt19937& rng, double scale) {
  std::normal_distribution<double> n(0.0, scale);
  Vector10 v;
  for (int i = 0; i < 10; ++i) v(i) = n(rng);
  return v;
}
State randomState(std::mt19937& rng) {
  return State(Gal3::Expmap(randomVector10(rng, 0.7)),
               randomVector10(rng, 0.1));
}
Group randomGroup(std::mt19937& rng) {
  return Group(Gal3::Expmap(randomVector10(rng, 0.7)),
               randomVector10(rng, 0.3));
}
}  // namespace

// φ is a right action: φ(φ(ξ, X), Y) = φ(ξ, X Y), φ(ξ, I) = ξ.
TEST(GalileanSymmetry, RightAction) {
  std::mt19937 rng(1);
  const Symmetry phi;
  for (int i = 0; i < 5; ++i) {
    const State xi = randomState(rng);
    const Group X = randomGroup(rng), Y = randomGroup(rng);
    EXPECT(assert_equal(phi(phi(xi, X), Y), phi(xi, X * Y), 1e-9));
    EXPECT(assert_equal(xi, phi(xi, Group::Identity()), 1e-12));
  }
}

TEST(GalileanSymmetry, ActionJacobians) {
  std::mt19937 rng(2);
  const Symmetry phi;
  const State xi = randomState(rng);
  const Group X = randomGroup(rng);
  Matrix20 Hxi, HX;
  phi(xi, X, Hxi, HX);
  const auto f = [&](const State& s, const Group& g) { return phi(s, g); };
  EXPECT(assert_equal(numericalDerivative21<State, State, Group>(f, xi, X),
                      Matrix(Hxi), 1e-6));
  EXPECT(assert_equal(numericalDerivative22<State, State, Group>(f, xi, X),
                      Matrix(HX), 1e-6));
}

// Dφ₀ = diag(I, -I), so the innovation lift S equals pinv(Dφ₀).
TEST(GalileanSymmetry, OriginDifferentialAndInnovationLift) {
  Matrix20 Dphi0;
  Symmetry::Orbit(State::Identity())(Group::Identity(), &Dphi0);
  Matrix20 S = Matrix20::Identity();
  S.bottomRightCorner<10, 10>() *= -1.0;
  EXPECT(assert_equal(Matrix(S), Matrix(Dphi0), 1e-12));

  std::mt19937 rng(3);
  Vector20 v;
  v << randomVector10(rng, 1.0), randomVector10(rng, 1.0);
  EXPECT(assert_equal(Vector(S * v), Vector(LiftInnovation(v)), 1e-12));
}

// The lift reproduces the exact discrete system: φ(ξ, Λ(ξ, u)) = f(ξ, u).
TEST(GalileanSymmetry, LiftReproducesDynamics) {
  std::mt19937 rng(4);
  for (int i = 0; i < 5; ++i) {
    const State xi = randomState(rng);
    EXPECT(assert_equal(Dynamics(xi, kImu, kEarth, 0.01),
                        Symmetry()(xi, LiftDiscrete(xi, kImu, kEarth, 0.01)),
                        1e-9));
  }
}

// With δ = 0, b = 0 and no rotation, eq. 25 is ordinary strapdown kinematics.
TEST(GalileanSymmetry, DynamicsIsStrapdownKinematics) {
  const Rot3 R = Rot3::RzRyRx(0.2, 0.1, -0.3);
  const Point3 p(1, 2, 3);
  const Vector3 v(0.5, -0.2, 1.0), a(0.4, 0.1, 9.5);
  const double dt = 0.1;
  const State next =
      Dynamics(State(Gal3(R, p, v, 0.0), Vector10::Zero()),
               IMUInput(Vector3::Zero(), a), EarthInput(kGravity, Z_3x1), dt);
  const Vector3 accWorld = R * a + kGravity;
  EXPECT(assert_equal(R, next.first.rotation(), 1e-12));
  EXPECT(
      assert_equal(Vector3(v + accWorld * dt), next.first.velocity(), 1e-12));
  EXPECT(assert_equal(Point3(p + v * dt + 0.5 * accWorld * dt * dt),
                      next.first.translation(), 1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, next.first.time(), 1e-12);
}

// Φ (eqs. 34-35) matches brute-force linearization of the error dynamics.
TEST(GalileanSymmetry, StateMatrixA) {
  std::mt19937 rng(5);
  const Group Xhat = randomGroup(rng);
  const State xi0 = State::Identity();
  const State xiHat = Symmetry()(xi0, Xhat);
  const Group XhatNext = Xhat * LiftDiscrete(xiHat, kImu, kEarth, kDt);

  const auto errorNext = [&](const Vector20& eps) -> Vector20 {
    const State xiTrue = Symmetry()(xi0.retract(eps), Xhat);
    const State xiTrueNext = Dynamics(xiTrue, kImu, kEarth, kDt);
    return xi0.localCoordinates(Symmetry()(xiTrueNext, XhatNext.inverse()));
  };
  const Matrix expected =
      numericalDerivative11<Vector20, Vector20>(errorNext, Vector20::Zero());
  const Matrix20 Phi =
      EqFStateMatrixA(Xhat, kEarth, kImu.extended() - bias(xiHat), kDt);
  EXPECT(assert_equal(expected, Matrix(Phi), 1e-6));
}

// B (eq. 36): driving IMU noise and bias random walk.
TEST(GalileanSymmetry, InputMatrixB) {
  std::mt19937 rng(6);
  const Group Xhat = randomGroup(rng);
  const State xi0 = State::Identity();
  const State xiHat = Symmetry()(xi0, Xhat);
  const Group XhatNext = Xhat * LiftDiscrete(xiHat, kImu, kEarth, kDt);

  // True system at zero error, driven by input minus noise η_w plus bias
  // random walk η_τ.
  const auto errorNext = [&](const Vector20& eta) -> Vector20 {
    State noisy = xiHat;
    noisy.second += eta.head<10>();  // w_N - η_w - b  ==  w_N - (b + η_w)
    State xiTrue = Dynamics(noisy, kImu, kEarth, kDt);
    xiTrue.second = xiHat.second + eta.tail<10>() * kDt;
    return xi0.localCoordinates(Symmetry()(xiTrue, XhatNext.inverse()));
  };
  const Matrix expected =
      numericalDerivative11<Vector20, Vector20>(errorNext, Vector20::Zero());
  const Matrix20 B =
      EqFInputMatrixB(Xhat, kEarth, kImu.extended() - bias(xiHat), kDt);
  EXPECT(assert_equal(expected, Matrix(B), 1e-6));
}

// The combined Φ/B evaluation matches the single-matrix APIs exactly.
TEST(GalileanSymmetry, StateAndInputMatricesShared) {
  std::mt19937 rng(8);
  for (int i = 0; i < 3; ++i) {
    const Group X = randomGroup(rng);  // nonidentity
    const Vector10 wTilde = randomVector10(rng, 1.0);
    for (double dt : {0.0, 0.005, 0.1}) {
      Matrix20 Phi, B;
      EqFStateAndInputMatrices(X, kEarth, wTilde, dt, Phi, B);
      EXPECT(assert_equal(Matrix(EqFStateMatrixA(X, kEarth, wTilde, dt)),
                          Matrix(Phi), 0.0));
      EXPECT(assert_equal(Matrix(EqFInputMatrixB(X, kEarth, wTilde, dt)),
                          Matrix(B), 0.0));
    }
  }
}

// GNSS output Jacobian w.r.t. F with Υ held fixed (wStart = 0), and C.
TEST(GalileanSymmetry, OutputJacobians) {
  std::mt19937 rng(7);
  const Gal3 F = Gal3::Expmap(randomVector10(rng, 0.5));
  const Gal3 Upsilon = Gal3::Expmap(randomVector10(rng, 0.3));
  const Point3 lever(0.1, -0.2, 0.05);

  Matrix3x10 H;
  GnssOutput(F, Upsilon, Vector10::Zero(), lever, H);
  const auto h = [&](const Gal3& f) {
    return GnssOutput(f, Upsilon, Vector10::Zero(), lever);
  };
  EXPECT(
      assert_equal(numericalDerivative11<Point3, Gal3>(h, F), Matrix(H), 1e-6));

  // C maps error coordinates at ξ₀ through the action at X.
  const Group X = randomGroup(rng);
  const Gal3 Fx = Symmetry()(State::Identity(), X).first;
  Matrix3x10 HFx;
  GnssOutput(Fx, Upsilon, Vector10::Zero(), lever, HFx);
  const auto hErr = [&](const Vector20& eps) {
    return GnssOutput(Symmetry()(State::Identity().retract(eps), X).first,
                      Upsilon, Vector10::Zero(), lever);
  };
  EXPECT(assert_equal(
      numericalDerivative11<Point3, Vector20>(hErr, Vector20::Zero()),
      Matrix(EqFOutputMatrixC(X, HFx)), 1e-6));
}

TEST(GalileanSymmetry, ResetJacobian) {
  std::mt19937 rng(8);
  Vector20 epsBar, deltaX;
  epsBar << randomVector10(rng, 0.05), randomVector10(rng, 0.05);
  deltaX << randomVector10(rng, 0.05), randomVector10(rng, 0.05);
  const auto reset = [&](const Vector20& eps) -> Vector20 {
    const State xi0 = State::Identity();
    return xi0.localCoordinates(
        Symmetry()(xi0.retract(eps), Group::Expmap(-deltaX)));
  };
  EXPECT(assert_equal(numericalDerivative11<Vector20, Vector20>(reset, epsBar),
                      Matrix(ResetJacobian(epsBar, deltaX)), 1e-6));
}

// Υ buffer interpolates exactly between IMU samples (zero-order hold).
TEST(GalileanSymmetry, UpsilonBufferInterpolation) {
  std::mt19937 rng(9);
  UpsilonBuffer buffer(1.0);
  std::vector<Vector10> inputs;
  const double dt = 0.01;
  for (int k = 0; k < 50; ++k) {
    inputs.push_back(randomVector10(rng, 0.5));
    buffer.push(inputs.back(), dt);
  }
  // δ = 0.127 s from t = 0.5 starts at 0.373: the last 0.007 s of sample 37
  // (held on [0.37, 0.38)), then samples 38..49 in full.
  Gal3 expected = Gal3::Expmap(inputs[37] * 0.007);
  for (int k = 38; k < 50; ++k)
    expected = expected * Gal3::Expmap(inputs[k] * dt);
  Vector10 wStart;
  EXPECT(assert_equal(expected, buffer.at(0.127, &wStart), 1e-9));
  EXPECT(assert_equal(inputs[37], wStart, 1e-12));
  EXPECT(assert_equal(Gal3(), buffer.at(0.0), 1e-12));
}

TEST(GalileanSymmetry, UpsilonBufferWindow) {
  UpsilonBuffer buffer(0.1);
  for (int k = 0; k < 100; ++k) buffer.push(Vector10::Zero(), 0.01);
  EXPECT(buffer.span() >= 0.1 - 1e-12);
  EXPECT(buffer.span() < 0.1 + 0.01 + 1e-12);
}

namespace {
/// Brute-force reference for the Υ buffer: full input history under
/// zero-order hold, integrated over [s, now] interval by interval.
struct ReferenceHistory {
  struct Interval {
    double start, dt;
    Vector10 input;
  };
  std::vector<Interval> intervals;
  double now = 0.0;

  void push(const Vector10& w, double dt) {
    intervals.push_back({now, dt, w});
    now += dt;  // same accumulation as UpsilonBuffer
  }

  /// Υ over [s, now] and the input held at s (later interval on a boundary,
  /// last interval at s == now).
  Gal3 upsilon(double s, Vector10* wStart) const {
    Gal3 result;
    *wStart = intervals.back().input;
    bool found = false;
    for (const Interval& in : intervals) {
      const double end = in.start + in.dt;
      if (!found && in.start <= s && s < end) {
        *wStart = in.input;
        found = true;
      }
      const double overlap = std::min(end, now) - std::max(in.start, s);
      if (overlap > 0.0) result = result * Gal3::Expmap(in.input * overlap);
    }
    return result;
  }
};
}  // namespace

// On-demand integration matches a brute-force reference with variable
// durations, pruning, partial intervals and out-of-range delays.
TEST(GalileanSymmetry, UpsilonBufferMatchesReference) {
  std::mt19937 rng(10);
  std::uniform_real_distribution<double> duration(0.002, 0.02);
  std::uniform_real_distribution<double> fraction(-0.2, 1.2);
  UpsilonBuffer buffer(0.15);
  ReferenceHistory reference;
  for (int k = 0; k < 200; ++k) {
    const Vector10 w = randomVector10(rng, 1.0);
    const double dt = duration(rng);
    buffer.push(w, dt);
    reference.push(w, dt);
    for (int q = 0; q < 5; ++q) {
      // Includes negative delays and delays beyond the available span.
      const double delta = fraction(rng) * buffer.span();
      const double s = reference.now - std::clamp(delta, 0.0, buffer.span());
      Vector10 wActual, wExpected;
      const Gal3 actual = buffer.at(delta, &wActual);
      const Gal3 expected = reference.upsilon(s, &wExpected);
      EXPECT(assert_equal(expected, actual, 1e-9));
      EXPECT(assert_equal(wExpected, wActual, 0.0));
    }
  }
  // Pruning keeps the buffer bounded: one interval reaches past the window.
  EXPECT(buffer.size() < 200);
  EXPECT(buffer.span() >= 0.15);
}

// Boundary cases, with dyadic durations so interval starts are exact.
TEST(GalileanSymmetry, UpsilonBufferBoundaries) {
  // Empty buffer: identity and zero input.
  {
    UpsilonBuffer empty(1.0);
    Vector10 w = Vector10::Ones();
    EXPECT(assert_equal(Gal3(), empty.at(0.1, &w), 1e-15));
    EXPECT(assert_equal(Vector10(Vector10::Zero()), w, 0.0));
  }

  std::mt19937 rng(12);
  const std::vector<double> durations = {0.0078125, 0.015625, 0.00390625,
                                         0.03125, 0.0078125};  // sum 0.06640625
  std::vector<Vector10> inputs;
  UpsilonBuffer buffer(1.0);
  for (double dt : durations) {
    inputs.push_back(randomVector10(rng, 1.0));
    buffer.push(inputs.back(), dt);
  }
  const double now = 0.06640625;
  EXPECT_DOUBLES_EQUAL(now, buffer.span(), 0.0);

  // Zero and negative delay: identity, but still the last input.
  Vector10 w;
  EXPECT(assert_equal(Gal3(), buffer.at(0.0, &w), 1e-15));
  EXPECT(assert_equal(inputs.back(), w, 0.0));
  EXPECT(assert_equal(Gal3(), buffer.at(-0.01, &w), 1e-15));
  EXPECT(assert_equal(inputs.back(), w, 0.0));

  // Exact internal boundary: selects the interval starting there, which is
  // then integrated in full.
  // Interval 3 starts at 0.0078125 + 0.015625 + 0.00390625 = 0.02734375.
  const double boundary = 0.02734375;
  Gal3 expected = Gal3::Expmap(inputs[3] * durations[3]) *
                  Gal3::Expmap(inputs[4] * durations[4]);
  EXPECT(assert_equal(expected, buffer.at(now - boundary, &w), 1e-12));
  EXPECT(assert_equal(inputs[3], w, 0.0));

  // Partial last interval only.
  EXPECT(assert_equal(Gal3::Expmap(inputs[4] * 0.00390625),
                      buffer.at(0.00390625, &w), 1e-12));
  EXPECT(assert_equal(inputs[4], w, 0.0));

  // Full span and beyond: clamped to the oldest interval.
  Gal3 all;
  for (size_t k = 0; k < inputs.size(); ++k)
    all = all * Gal3::Expmap(inputs[k] * durations[k]);
  EXPECT(assert_equal(all, buffer.at(now, &w), 1e-12));
  EXPECT(assert_equal(inputs[0], w, 0.0));
  EXPECT(assert_equal(all, buffer.at(10.0, &w), 1e-12));
  EXPECT(assert_equal(inputs[0], w, 0.0));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}