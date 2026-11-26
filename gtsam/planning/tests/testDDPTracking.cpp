// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file testDDPTracking.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/planning/DDPTracking.h>
#include <gtsam/planning/ILQRTracking.h>

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

// On a linear system (zero dynamics Hessian), DDPTracking matches ILQRTracking
// across controls, states, gains, and eliminated GaussianBayesNet policy.
TEST(DDPTracking, MatchesILQROnLinearSystem) {
  const double dt = 0.02;
  const size_t h = 10;
  const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
  const auto doubleIntegrator = [dt](const Vector& x,
                                     const Vector& u) -> Vector {
    return Vector4(x[0] + dt * x[2], x[1] + dt * x[3], x[2] + dt * u[0],
                   x[3] + dt * u[1]);
  };
  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(X(0), Vector(Vector4(0.3, -0.2, 0.5, -0.4)));
  const auto qNoise =
      noiseModel::Diagonal::Precisions(dt * Vector4(500., 500., 100., 100.));
  const auto rNoise = noiseModel::Isotropic::Precision(2, dt);
  const auto termNoise = noiseModel::Unit::Create(4);
  for (size_t k = 0; k < h; ++k) {
    initial.insert(U(k), Vector(Vector2::Zero()));
    graph.addPrior(X(k), Vector(Vector4::Zero()), qNoise);
    graph.addPrior(U(k), Vector(Vector2::Zero()), rNoise);
  }
  graph.addPrior(X(h), Vector(Vector4::Zero()), termNoise);

  ILQRTracking<> ilqr(xKeys, uKeys);
  DDPTracking<> ddp(xKeys, uKeys);
  const Values planIlqr = ilqr.optimize(graph, initial, doubleIntegrator, 5);
  const Values planDdp = ddp.optimize(graph, initial, doubleIntegrator, 5);

  EXPECT_LONGS_EQUAL(static_cast<long>(h),
                     static_cast<long>(ddp.policy().size()));
  for (size_t k = 0; k < h; ++k) {
    EXPECT(assert_equal(planDdp.at<Vector>(U(k)), planIlqr.at<Vector>(U(k)),
                        1e-5));
    EXPECT(assert_equal(planDdp.at<Vector>(X(k + 1)),
                        planIlqr.at<Vector>(X(k + 1)), 1e-5));
    EXPECT(assert_equal(ddp.gain(k), ilqr.gain(k), 1e-5));
    const auto& cond = ddp.policy()[k];
    const Matrix kFromBn =
        -cond->R().triangularView<Eigen::Upper>().solve(cond->S());
    EXPECT(assert_equal(ddp.gain(k), kFromBn, 1e-12));
  }
}

// On a nonlinear pendulum swing-up and feasible-reference tracking, full
// second-order DDPTracking agrees with ILQRTracking.
TEST(DDPTracking, MatchesILQROnNonlinearPendulum) {
  const double dt = 0.05;
  const size_t h = 25;
  const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
  const auto field = [](const Vector& x, const Vector& u) -> Vector {
    return Vector2(x[1], -9.81 * std::sin(x[0]) + u[0]);
  };
  const auto pendulum = [dt](const Vector& x, const Vector& u) -> Vector {
    return Vector2(x[0] + dt * x[1],
                   x[1] + dt * (-9.81 * std::sin(x[0]) + u[0]));
  };
  const auto qNoise = noiseModel::Diagonal::Precisions(Vector2(2., 0.2));
  const auto rNoise = noiseModel::Isotropic::Precision(1, 0.1);
  const auto termNoise = noiseModel::Diagonal::Precisions(Vector2(20., 2.));

  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(X(0), Vector(Vector2(0.2, 0.0)));
  const Vector goal = Vector2(M_PI, 0.0);
  for (size_t k = 0; k < h; ++k) {
    initial.insert(U(k), Vector(Vector1{0.}));
    graph.addPrior(X(k), goal, qNoise);
    graph.addPrior(U(k), Vector(Vector1{0.}), rNoise);
  }
  graph.addPrior(X(h), goal, termNoise);

  ILQRTracking<> ilqr(xKeys, uKeys);
  DDPTracking<> ddp(xKeys, uKeys), ddpCont(xKeys, uKeys);
  const Values planIlqr = ilqr.optimize(graph, initial, pendulum, 50);
  const Values planDdp = ddp.optimize(graph, initial, pendulum, 50);
  const Values planCont = ddpCont.optimize(graph, initial, field, dt, 50);
  for (size_t k = 0; k < h; ++k) {
    EXPECT(assert_equal(planDdp.at<Vector>(U(k)), planCont.at<Vector>(U(k)),
                        1e-8));
    EXPECT(assert_equal(planDdp.at<Vector>(U(k)), planIlqr.at<Vector>(U(k)),
                        1e-2));
    EXPECT(assert_equal(planDdp.at<Vector>(X(k + 1)),
                        planIlqr.at<Vector>(X(k + 1)), 1e-2));
  }

  // Tracking the feasible nonlinear reference from a perturbed initial state.
  NonlinearFactorGraph trackGraph;
  Values trackInit = planDdp;
  trackInit.update(X(0), Vector(Vector2(0.25, -0.05)));
  for (size_t k = 0; k < h; ++k) {
    trackGraph.addPrior(X(k), planDdp.at<Vector>(X(k)), qNoise);
    trackGraph.addPrior(U(k), planDdp.at<Vector>(U(k)), rNoise);
  }
  trackGraph.addPrior(X(h), planDdp.at<Vector>(X(h)), termNoise);

  ILQRTracking<> trackIlqr(xKeys, uKeys);
  DDPTracking<> trackDdp(xKeys, uKeys);
  const Values trackPlanIlqr =
      trackIlqr.optimize(trackGraph, trackInit, pendulum, 50);
  const Values trackPlanDdp =
      trackDdp.optimize(trackGraph, trackInit, pendulum, 50);
  for (size_t k = 0; k < h; ++k) {
    EXPECT(assert_equal(trackPlanDdp.at<Vector>(U(k)),
                        trackPlanIlqr.at<Vector>(U(k)), 1e-2));
  }
  EXPECT(assert_equal(trackDdp.gain(0), trackIlqr.gain(0), 1e-1));
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

// DDPTracking works on Lie groups (Pose2) and non-Lie manifolds (Unit3 on S^2)
// with NonlinearFactorGraph objectives and agrees with ILQRTracking.
TEST(DDPTracking, ManifoldPose2AndUnit3Tracking) {
  // Pose2 unicycle tracking.
  {
    const double dt = 0.1;
    const size_t h = 20;
    const auto xKeys = stateKeys(h), uKeys = controlKeys(h);
    const auto discreteExpmap = [dt](const Pose2& x, const Vector& u) -> Pose2 {
      return x.compose(Pose2::Expmap(Vector3(dt * u[0], 0.0, dt * u[1])));
    };
    const Vector uNominal = Vector2(1.0, 0.4);
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

    ILQRTracking<Pose2> ilqr(xKeys, uKeys);
    DDPTracking<Pose2> ddp(xKeys, uKeys);
    const Values planIlqr = ilqr.optimize(graph, initial, discreteExpmap, 25);
    const Values planDdp = ddp.optimize(graph, initial, discreteExpmap, 25);
    for (size_t k = 0; k < h; ++k) {
      EXPECT(assert_equal(planIlqr.at<Vector>(U(k)), planDdp.at<Vector>(U(k)),
                          1e-3));
    }
    EXPECT_LONGS_EQUAL(static_cast<long>(h),
                       static_cast<long>(ddp.policy().size()));
  }

  // Unit3 tracking on S^2.
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

    DDPTracking<Unit3> solver(xKeys, uKeys);
    const Values plan = solver.optimize(graph, initial, field, dt, 20);
    EXPECT(traits<Unit3>::Local(goal, plan.at<Unit3>(X(h))).norm() < 0.02);
    EXPECT_LONGS_EQUAL(static_cast<long>(h),
                       static_cast<long>(solver.policy().size()));
  }
}

}  // namespace manifold
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
