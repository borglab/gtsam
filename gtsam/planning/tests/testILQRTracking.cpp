// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file testILQRTracking.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/planning/DDPTracking.h>
#include <gtsam/planning/ILQRTracking.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <vector>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace euclidean {

KeyVector stateKeys(size_t horizon) {
  KeyVector keys;
  keys.reserve(horizon + 1);
  for (size_t k = 0; k <= horizon; ++k) keys.push_back(X(k));
  return keys;
}

KeyVector controlKeys(size_t horizon) {
  KeyVector keys;
  keys.reserve(horizon);
  for (size_t k = 0; k < horizon; ++k) keys.push_back(U(k));
  return keys;
}

// A one-step linear-quadratic problem has a closed-form optimal input, gain,
// and eliminated GaussianBayesNet conditional.
TEST(ILQRTracking, OneStepLinearQuadraticAndBounds) {
  const auto xKeys = stateKeys(1), uKeys = controlKeys(1);
  NonlinearFactorGraph graph;
  graph.addPrior(U(0), Vector(Vector1{0.}), noiseModel::Unit::Create(1));
  graph.addPrior(X(1), Vector(Vector1{2.}),
                 noiseModel::Isotropic::Precision(1, 3.));
  Values initial;
  initial.insert(X(0), Vector(Vector1{0.}));
  initial.insert(U(0), Vector(Vector1{0.}));

  ILQRTracking<> solver(xKeys, uKeys);
  const auto step = [](const Vector& state, const Vector& ctrl) -> Vector {
    return state + ctrl;
  };
  Values plan = solver.optimize(graph, initial, step, 2);
  EXPECT_DOUBLES_EQUAL(1.5, plan.at<Vector>(U(0))[0], 1e-6);
  EXPECT_DOUBLES_EQUAL(1.5, plan.at<Vector>(X(1))[0], 1e-6);
  EXPECT_DOUBLES_EQUAL(-0.75, solver.gain(0)(0, 0), 1e-6);
  EXPECT_LONGS_EQUAL(1L, static_cast<long>(solver.policy().size()));

  const auto& cond = solver.policy()[0];
  const Matrix kFromBn =
      -cond->R().triangularView<Eigen::Upper>().solve(cond->S());
  EXPECT(assert_equal(solver.gain(0), kFromBn, 1e-12));

  // Control bounds clamp during forward rollout.
  plan = solver.optimize(graph, initial, step, 2, Vector1{-0.5}, Vector1{0.5});
  EXPECT_DOUBLES_EQUAL(0.5, plan.at<Vector>(U(0))[0], 1e-10);
  EXPECT_DOUBLES_EQUAL(0.5, plan.at<Vector>(X(1))[0], 1e-10);
}

// A linear trajectory problem can also be solved by sequential QR elimination
// of the full graph. Both trackers should recover its controls and states in
// one forward substitution, including feedback from earlier control changes.
TEST(ILQRTracking, BackSubstitutionMatchesFullGaussianGraph) {
  const auto xs = stateKeys(3), us = controlKeys(3);
  const Vector zero = Vector1::Zero();
  const Matrix identity = Matrix::Identity(1, 1);
  const auto hard = noiseModel::Constrained::All(1);
  NonlinearFactorGraph cost;
  Values nominal;
  nominal.insert(xs.front(), zero);
  for (size_t k = 0; k < us.size(); ++k) {
    nominal.insert(us[k], zero);
    nominal.insert(xs[k + 1], zero);
    cost.addPrior(us[k], zero);
  }
  cost.addPrior(xs.back(), Vector(Vector1{4.0}));

  auto graph = cost.linearize(nominal);
  graph->emplace_shared<JacobianFactor>(xs.front(), identity, zero, hard);
  for (size_t k = 0; k < us.size(); ++k)
    graph->emplace_shared<JacobianFactor>(xs[k], identity, us[k], identity,
                                          xs[k + 1], -identity, zero, hard);
  const Ordering order{X(3), U(2), X(2), U(1), X(1), U(0), X(0)};
  const VectorValues expected =
      graph->eliminateSequential(order, EliminateQR)->optimize();
  const auto step = [](const Vector& x, const Vector& u) -> Vector {
    return x + u;
  };

  auto check = [&](auto& tracker) {
    tracker.optimize(cost, nominal, step, 0);
    for (size_t k = 0; k < us.size(); ++k) {
      const VectorValues parent{{xs[k], expected.at(xs[k])}};
      EXPECT(assert_equal(expected.at(us[k]),
                          tracker.policy()[k]->solve(parent).at(us[k]), 1e-8));
    }
    const Values plan = tracker.optimize(cost, nominal, step, 1);
    for (size_t k = 0; k < us.size(); ++k) {
      EXPECT(assert_equal(expected.at(us[k]), plan.at<Vector>(us[k]), 1e-8));
      EXPECT(assert_equal(expected.at(xs[k + 1]), plan.at<Vector>(xs[k + 1]),
                          1e-8));
    }
  };
  ILQRTracking<> ilqr(xs, us);
  DDPTracking<> ddp(xs, us);
  check(ilqr);
  check(ddp);
}

// Cached correlated priors and ordinary factors must agree with the original
// graph at each new nominal trajectory, including the Gaussian constant term.
TEST(ILQRTracking, MixedCostGraphMatchesDirectLinearization) {
  const KeyVector xs{X(0), X(1)}, us{U(0)};
  const auto stages = internal::buildStageKeyMap(xs, us);
  const auto gaussian =
      noiseModel::Gaussian::Information(Matrix2{{3, 1}, {1, 2}});
  const auto robust = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(0.5), gaussian);
  NonlinearFactorGraph graph;
  graph.addPrior(U(0), Vector2{1, 2}, gaussian);
  graph.addPrior(U(0), Vector2{-1, 3}, gaussian);
  graph.addPrior(X(1), Vector2{2, -1}, gaussian);
  graph.addPrior(X(1), Vector2{-2, 1}, robust);
  graph.emplace_shared<BetweenFactor<Vector2>>(X(0), U(0), Vector2{1, -1},
                                               gaussian);
  graph.push_back(NonlinearFactor::shared_ptr{});

  Values initial;
  initial.insert(X(0), Vector2{1, 1});
  initial.insert(42, 1.25);
  internal::TrackingObjective<Vector2, Vector2> cost(graph, initial, xs, us,
                                                     stages, 2, 2);
  VectorValues delta;
  delta.insert(X(0), Vector2{0.1, 0.2});
  delta.insert(U(0), Vector2{-0.3, 0.4});
  delta.insert(X(1), Vector2{0.5, -0.6});
  for (double offset : {0.0, 2.0}) {
    const std::vector<Vector2> states{{1, 1}, {offset, -2}};
    const Vector2 control{offset, 3};
    const Values values = cost.trajectoryValues(states, control);
    EXPECT_DOUBLES_EQUAL(1.25, values.at<double>(42), 0.0);
    EXPECT_DOUBLES_EQUAL(graph.error(values), cost.error(states, control),
                         1e-10);
    const auto local = cost.linearize(states, control);
    EXPECT_DOUBLES_EQUAL(graph.linearize(values)->error(delta),
                         local.stageFactors.front().error(delta) +
                             local.terminalFactor.error(delta),
                         1e-10);
  }
}

// Analytic OptionalJacobian transitions match finite-difference ILQRTracking
// and continuous (vectorField, dt) overloads match discrete Euler steps.
TEST(ILQRTracking, AnalyticJacobiansAndVectorField) {
  const double dt = 0.05;
  const size_t h = 25;
  const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
  const auto field = [](const Vector& x, const Vector& u) -> Vector {
    return Vector2(x[1], -9.81 * std::sin(x[0]) + u[0]);
  };
  const auto discrete = [dt](const Vector& x, const Vector& u) -> Vector {
    return Vector2(x[0] + dt * x[1],
                   x[1] + dt * (-9.81 * std::sin(x[0]) + u[0]));
  };
  const ILQRTracking<>::LinearizedTransition analytic =
      [dt](const Vector& x, const Vector& u, ILQRTracking<>::OptionalMatrix Hx,
           ILQRTracking<>::OptionalMatrix Hu) -> Vector {
    if (Hx) {
      *Hx = Matrix2{{1.0, dt}, {-dt * 9.81 * std::cos(x[0]), 1.0}};
    }
    if (Hu) {
      *Hu = Vector2(0.0, dt);
    }
    return Vector2(x[0] + dt * x[1],
                   x[1] + dt * (-9.81 * std::sin(x[0]) + u[0]));
  };

  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(X(0), Vector(Vector2(0.2, 0.0)));
  const auto qNoise = noiseModel::Diagonal::Precisions(Vector2(2., 0.2));
  const auto rNoise = noiseModel::Isotropic::Precision(1, 0.1);
  const auto termNoise = noiseModel::Diagonal::Precisions(Vector2(20., 2.));
  const Vector goal = Vector2(M_PI, 0.0);
  for (size_t k = 0; k < h; ++k) {
    initial.insert(U(k), Vector(Vector1{0.}));
    graph.addPrior(X(k), goal, qNoise);
    graph.addPrior(U(k), Vector(Vector1{0.}), rNoise);
  }
  graph.addPrior(X(h), goal, termNoise);

  ILQRTracking<> ilqrFd(xKeys, uKeys), ilqrAna(xKeys, uKeys),
      ilqrCont(xKeys, uKeys);
  const Values planFd = ilqrFd.optimize(graph, initial, discrete, 50);
  const Values planAna = ilqrAna.optimize(graph, initial, analytic, 50);
  const Values planCont = ilqrCont.optimize(graph, initial, field, dt, 50);
  for (size_t k = 0; k < h; ++k) {
    EXPECT(
        assert_equal(planFd.at<Vector>(U(k)), planAna.at<Vector>(U(k)), 1e-4));
    EXPECT(
        assert_equal(planFd.at<Vector>(U(k)), planCont.at<Vector>(U(k)), 1e-8));
  }
  EXPECT(assert_equal(ilqrFd.gain(0), ilqrAna.gain(0), 1e-3));
}

}  // namespace euclidean
/* ************************************************************************* */

/* ************************************************************************* */
namespace manifold {

KeyVector stateKeys(size_t horizon) {
  KeyVector keys;
  keys.reserve(horizon + 1);
  for (size_t k = 0; k <= horizon; ++k) keys.push_back(X(k));
  return keys;
}

KeyVector controlKeys(size_t horizon) {
  KeyVector keys;
  keys.reserve(horizon);
  for (size_t k = 0; k < horizon; ++k) keys.push_back(U(k));
  return keys;
}

// ILQRTracking<Pose2, Vector2> tracks a Pose2 unicycle reference using a
// NonlinearFactorGraph objective, matching analytic OptionalJacobian, FD
// discrete, and continuous (vectorField, dt) interfaces.
TEST(ILQRTracking, ManifoldPose2Tracking) {
  const double dt = 0.1;
  const size_t h = 20;
  const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
  const auto field = [](const Pose2&, const Vector2& u) -> Vector {
    return Vector3(u[0], 0.0, u[1]);
  };
  const auto discreteExpmap = [dt](const Pose2& x, const Vector2& u) -> Pose2 {
    return x.compose(Pose2::Expmap(Vector3(dt * u[0], 0.0, dt * u[1])));
  };
  const ILQRTracking<Pose2, Vector2>::LinearizedTransition analytic =
      [dt](const Pose2& x, const Vector2& u,
           ILQRTracking<Pose2, Vector2>::OptionalMatrix Hx,
           ILQRTracking<Pose2, Vector2>::OptionalMatrix Hu) -> Pose2 {
    const Vector3 xi(dt * u[0], 0.0, dt * u[1]);
    Matrix3 Dexp;
    const Pose2 delta = Pose2::Expmap(xi, Hu ? &Dexp : nullptr);
    Matrix3 H1, H2;
    const Pose2 next = x.compose(delta, Hx ? &H1 : nullptr, Hu ? &H2 : nullptr);
    if (Hx) *Hx = H1;
    if (Hu) {
      Matrix32 Dxi = Matrix32::Zero();
      Dxi(0, 0) = dt;
      Dxi(2, 1) = dt;
      *Hu = H2 * Dexp * Dxi;
    }
    return next;
  };

  // Build a feasible reference curve on SE(2) from constant forward + yaw.
  const Vector2 uNominal{1.0, 0.4};
  std::vector<Pose2> xRef(h + 1);
  xRef[0] = Pose2(0.0, 0.0, 0.0);
  for (size_t k = 0; k < h; ++k)
    xRef[k + 1] = discreteExpmap(xRef[k], uNominal);

  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(X(0), Pose2(0.0, -0.15, 0.1));
  const auto qNoise = noiseModel::Diagonal::Precisions(Vector3(5., 5., 2.));
  const auto rNoise = noiseModel::Diagonal::Precisions(Vector2(0.2, 0.2));
  const auto termNoise =
      noiseModel::Diagonal::Precisions(Vector3(20., 20., 10.));
  for (size_t k = 0; k < h; ++k) {
    initial.insert(U(k), uNominal);
    graph.addPrior(X(k), xRef[k], qNoise);
    graph.addPrior(U(k), uNominal, rNoise);
  }
  graph.addPrior(X(h), xRef[h], termNoise);

  ILQRTracking<Pose2, Vector2> ilqrAna(xKeys, uKeys), ilqrFd(xKeys, uKeys),
      ilqrCont(xKeys, uKeys);
  const Values planAna = ilqrAna.optimize(graph, initial, analytic, 25);
  const Values planFd = ilqrFd.optimize(graph, initial, discreteExpmap, 25);
  const Values planCont = ilqrCont.optimize(graph, initial, field, dt, 25);

  EXPECT_LONGS_EQUAL(static_cast<long>(h),
                     static_cast<long>(ilqrAna.policy().size()));
  for (size_t k = 0; k < h; ++k) {
    EXPECT(assert_equal(planAna.atRef<Vector2>(U(k)),
                        planFd.atRef<Vector2>(U(k)), 1e-5));
    const auto& cond = ilqrAna.policy()[k];
    const Matrix kFromBn =
        -cond->R().triangularView<Eigen::Upper>().solve(cond->S());
    EXPECT(assert_equal(ilqrAna.gain(k), kFromBn, 1e-12));
  }
  EXPECT(assert_equal(ilqrAna.gain(0), ilqrFd.gain(0), 1e-4));
  EXPECT(traits<Pose2>::Local(xRef.back(), planAna.at<Pose2>(X(h))).norm() <
         0.05);
  EXPECT(traits<Pose2>::Local(xRef.back(), planCont.at<Pose2>(X(h))).norm() <
         0.05);
}

// ILQRTracking works on non-Lie-group manifolds (Unit3 on S^2) and 3D
// rotations (Rot3 on SO(3)) with exact NonlinearFactorGraph linearization.
TEST(ILQRTracking, ManifoldUnit3AndRot3Tracking) {
  // Unit3 tracking in the moving tangent plane of S^2.
  {
    const double dt = 0.1;
    const size_t h = 15;
    const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
    const auto field = [](const Unit3&, const Vector& u) -> Vector {
      return Vector2(u[0], u[1]);
    };
    const Unit3 start(1.0, 0.0, 0.0);
    const Unit3 goal(1.0, 0.3, -0.2);
    NonlinearFactorGraph graph;
    Values initial;
    initial.insert(X(0), start);
    const auto qNoise = noiseModel::Unit::Create(2);
    const auto rNoise = noiseModel::Isotropic::Precision(2, 0.05);
    const auto termNoise = noiseModel::Isotropic::Precision(2, 100.0);
    for (size_t k = 0; k < h; ++k) {
      initial.insert(U(k), Vector(Vector2::Zero()));
      graph.addPrior(X(k), goal, qNoise);
      graph.addPrior(U(k), Vector(Vector2::Zero()), rNoise);
    }
    graph.addPrior(X(h), goal, termNoise);

    ILQRTracking<Unit3> solver(xKeys, uKeys);
    const Values plan = solver.optimize(graph, initial, field, dt, 20);
    EXPECT(traits<Unit3>::Local(goal, plan.at<Unit3>(X(h))).norm() < 0.02);
  }

  // Rot3 attitude tracking on SO(3).
  {
    const double dt = 0.1;
    const size_t h = 15;
    const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
    const auto field = [](const Rot3&, const Vector& omega) -> Vector {
      return omega;
    };
    const Rot3 goal = Rot3::Ypr(0.4, -0.2, 0.3);
    NonlinearFactorGraph graph;
    Values initial;
    initial.insert(X(0), Rot3::Identity());
    const auto qNoise = noiseModel::Unit::Create(3);
    const auto rNoise = noiseModel::Isotropic::Precision(3, 0.05);
    const auto termNoise = noiseModel::Isotropic::Precision(3, 100.0);
    for (size_t k = 0; k < h; ++k) {
      initial.insert(U(k), Vector(Vector3::Zero()));
      graph.addPrior(X(k), goal, qNoise);
      graph.addPrior(U(k), Vector(Vector3::Zero()), rNoise);
    }
    graph.addPrior(X(h), goal, termNoise);

    ILQRTracking<Rot3> solver(xKeys, uKeys);
    const Values plan = solver.optimize(graph, initial, field, dt, 20);
    EXPECT(traits<Rot3>::Local(goal, plan.at<Rot3>(X(h))).norm() < 0.02);
  }
}

}  // namespace manifold
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
