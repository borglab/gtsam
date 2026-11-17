/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStagCost.cpp
 * @brief   Unit tests for the STAG cost base class
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/mpc_stag/cost/StagCost.h>

#include <memory>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace goal_cost {

using Cost = StagCost<Pose2, Vector2>;

/**
 * Goal cost on Pose2 / Vector2: the running cost is a prior on x_k at the
 * goal (weights Q) and a prior on u_k at zero (weights R), and the terminal
 * cost is a prior on x_H at the goal (weights Qf).
 */
class GoalCost : public Cost {
 public:
  /// Construct from the goal and the weights Q, R and Qf.
  GoalCost(const Pose2& goal, const Matrix3& Q, const Matrix2& R,
           const Matrix3& Qf)
      : goal_(goal),
        stateNoise_(noiseModel::Gaussian::Information(Q)),
        controlNoise_(noiseModel::Gaussian::Information(R)),
        terminalNoise_(noiseModel::Gaussian::Information(Qf)) {}

  /// Priors on x_k at the goal and on u_k at zero.
  NonlinearFactorGraph runningCost(size_t /*k*/, Key stateKey, Key controlKey,
                                   double /*time*/) const override {
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<Pose2>>(stateKey, goal_, stateNoise_);
    graph.emplace_shared<PriorFactor<Vector2>>(controlKey, Vector2{0.0, 0.0},
                                               controlNoise_);
    return graph;
  }

  /// Prior on x_H at the goal.
  NonlinearFactorGraph terminalCost(Key stateKey,
                                    double /*time*/) const override {
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<Pose2>>(stateKey, goal_, terminalNoise_);
    return graph;
  }

 private:
  Pose2 goal_;
  SharedNoiseModel stateNoise_, controlNoise_, terminalNoise_;
};

const Pose2 goal(5.0, 3.0, 0.5);
const Matrix3 Q{{2.0, 0.5, 0.0}, {0.5, 1.0, 0.0}, {0.0, 0.0, 4.0}};
const Matrix2 R{{0.1, 0.0}, {0.0, 0.2}};
const Matrix3 Qf{{10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 20.0}};
const GoalCost goalCost(goal, Q, R, Qf);
const Cost& cost = goalCost;  // all calls go through the base interface

/// 1/2 ||Local(goal, x)||^2_W.
double stateCost(const Pose2& x, const Matrix3& W) {
  const Vector3 e = traits<Pose2>::Local(goal, x);
  return 0.5 * e.dot(W * e);
}

// Checks that the running cost of step k is a prior on x_k and one on u_k.
TEST(StagCost, RunningCostFactors) {
  const NonlinearFactorGraph graph = cost.runningCost(2, X(2), U(2), 0.2);
  EXPECT_LONGS_EQUAL(2, graph.size());
  EXPECT(graph.at(0)->keys() == KeyVector{X(2)});
  EXPECT(graph.at(1)->keys() == KeyVector{U(2)});
  EXPECT(std::dynamic_pointer_cast<PriorFactor<Pose2>>(graph.at(0)));
  EXPECT(std::dynamic_pointer_cast<PriorFactor<Vector2>>(graph.at(1)));
}

// Checks that the terminal cost is one prior on x_H.
TEST(StagCost, TerminalCostFactors) {
  const NonlinearFactorGraph graph = cost.terminalCost(X(4), 0.4);
  EXPECT_LONGS_EQUAL(1, graph.size());
  EXPECT(graph.at(0)->keys() == KeyVector{X(4)});
  EXPECT(std::dynamic_pointer_cast<PriorFactor<Pose2>>(graph.at(0)));
}

// Checks that the factors' error is 1/2 ||Local(goal, x)||^2_Q + 1/2 ||u||^2_R
// for the running cost and 1/2 ||Local(goal, x)||^2_Qf for the terminal cost.
TEST(StagCost, Error) {
  const Pose2 x(1.0, 2.0, 0.3);
  const Vector2 u{1.5, -0.4};
  Values values;
  values.insert(X(0), x);
  values.insert(U(0), u);

  const double expectedRunning = stateCost(x, Q) + 0.5 * u.dot(R * u);
  EXPECT_DOUBLES_EQUAL(expectedRunning,
                       cost.runningCost(0, X(0), U(0), 0.0).error(values),
                       1e-9);
  EXPECT_DOUBLES_EQUAL(stateCost(x, Qf),
                       cost.terminalCost(X(0), 0.0).error(values), 1e-9);
}

// Checks that the running costs of steps 0 to H-1 and the terminal cost,
// pushed into one graph, give 2H + 1 factors and the sum of their errors.
TEST(StagCost, FullHorizon) {
  const size_t H = 4;
  const double time = 1.0, dt = 0.1;
  Values values;
  for (size_t k = 0; k <= H; ++k) {
    values.insert(X(k), Pose2(1.0 + k, 2.0 - 0.5 * k, 0.1 * k));
  }
  for (size_t k = 0; k < H; ++k) {
    values.insert(U(k), Vector2{0.5 * k, -0.2 + 0.1 * k});
  }

  NonlinearFactorGraph graph;
  double sumOfParts = 0.0;
  for (size_t k = 0; k < H; ++k) {
    const NonlinearFactorGraph running =
        cost.runningCost(k, X(k), U(k), time + k * dt);
    graph.push_back(running);
    sumOfParts += running.error(values);
  }
  const NonlinearFactorGraph terminal = cost.terminalCost(X(H), time + H * dt);
  graph.push_back(terminal);
  sumOfParts += terminal.error(values);

  EXPECT_LONGS_EQUAL(2 * H + 1, graph.size());
  EXPECT_DOUBLES_EQUAL(sumOfParts, graph.error(values), 1e-9);
}

}  // namespace goal_cost
/* ************************************************************************* */

/* ************************************************************************* */
namespace running_only_cost {

/// A cost that overrides only runningCost: a prior on u_k at zero.
class RunningOnlyCost : public StagCost<Pose2, Vector2> {
 public:
  /// A unit prior on u_k at zero.
  NonlinearFactorGraph runningCost(size_t /*k*/, Key /*stateKey*/,
                                   Key controlKey,
                                   double /*time*/) const override {
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<Vector2>>(
        controlKey, Vector2{0.0, 0.0}, noiseModel::Unit::Create(2));
    return graph;
  }
};

// Checks that a cost that does not override terminalCost returns an empty
// graph.
TEST(StagCost, DefaultTerminalCostIsEmpty) {
  const RunningOnlyCost runningOnly;
  const StagCost<Pose2, Vector2>& cost = runningOnly;
  EXPECT(cost.terminalCost(X(3), 0.3).empty());
  EXPECT_LONGS_EQUAL(1, cost.runningCost(0, X(0), U(0), 0.0).size());
}

}  // namespace running_only_cost
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
