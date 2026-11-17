/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStagQuadraticCost.cpp
 * @brief   Unit tests for the quadratic state and control cost
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/mpc_stag/cost/StagQuadraticCost.h>

#include <memory>
#include <type_traits>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace pose2_quadratic_cost {

using Cost = StagQuadraticCost<Pose2, Vector2>;

const Pose2 goal(5.0, 3.0, 0.5);
const Matrix3 Q{{2.0, 0.5, 0.0}, {0.5, 1.0, 0.0}, {0.0, 0.0, 4.0}};
const Matrix2 R{{0.1, 0.0}, {0.0, 0.2}};
const Matrix3 Qf{{10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {0.0, 0.0, 20.0}};
const Cost quadraticCost(goal, Q, R, Qf);
const StagCost<Pose2, Vector2>& cost = quadraticCost;  // the base interface

const Pose2 x(1.0, 2.0, 0.3);
const Vector2 u{1.5, -0.4};

/// x_0 = x and u_0 = u.
Values testValues() {
  Values values;
  values.insert(X(0), x);
  values.insert(U(0), u);
  return values;
}

/// 1/2 ||Local(goal, x)||^2_W.
double stateCost(const Pose2& state, const Matrix3& W) {
  const Vector3 e = traits<Pose2>::Local(goal, state);
  return 0.5 * e.dot(W * e);
}

// Checks the dimensions and the fixed-size weight types.
TEST(StagQuadraticCost, Dimensions) {
  EXPECT_LONGS_EQUAL(3, Cost::StateDim);
  EXPECT_LONGS_EQUAL(2, Cost::ControlDim);
  EXPECT((std::is_same_v<Cost::StateWeight, Matrix3>));
  EXPECT((std::is_same_v<Cost::ControlWeight, Matrix2>));
}

// Checks that the running cost is a prior on x_k and a prior on u_k.
TEST(StagQuadraticCost, RunningCostFactors) {
  const NonlinearFactorGraph graph = cost.runningCost(2, X(2), U(2), 0.2);
  EXPECT_LONGS_EQUAL(2, graph.size());
  EXPECT(graph.at(0)->keys() == KeyVector{X(2)});
  EXPECT(graph.at(1)->keys() == KeyVector{U(2)});
  EXPECT(std::dynamic_pointer_cast<PriorFactor<Pose2>>(graph.at(0)));
  EXPECT(std::dynamic_pointer_cast<PriorFactor<Vector2>>(graph.at(1)));
}

// Checks that the terminal cost is one prior on x_H.
TEST(StagQuadraticCost, TerminalCostFactors) {
  const NonlinearFactorGraph graph = cost.terminalCost(X(4), 0.4);
  EXPECT_LONGS_EQUAL(1, graph.size());
  EXPECT(graph.at(0)->keys() == KeyVector{X(4)});
  EXPECT(std::dynamic_pointer_cast<PriorFactor<Pose2>>(graph.at(0)));
}

// Checks that the running cost is 1/2 ||Local(goal, x)||^2_Q + 1/2 ||u||^2_R.
TEST(StagQuadraticCost, RunningCostError) {
  const Values values = testValues();
  EXPECT_DOUBLES_EQUAL(stateCost(x, Q) + 0.5 * u.dot(R * u),
                       cost.runningCost(0, X(0), U(0), 0.0).error(values),
                       1e-9);
}

// Checks that the terminal cost is 1/2 ||Local(goal, x)||^2_Qf.
TEST(StagQuadraticCost, TerminalCostError) {
  const Values values = testValues();
  EXPECT_DOUBLES_EQUAL(stateCost(x, Qf),
                       cost.terminalCost(X(0), 0.0).error(values), 1e-9);
}

// Checks that both costs are zero at the goal with zero control.
TEST(StagQuadraticCost, ZeroAtGoal) {
  Values atGoal;
  atGoal.insert(X(0), goal);
  atGoal.insert(U(0), Vector2{0.0, 0.0});
  EXPECT_DOUBLES_EQUAL(0.0, cost.runningCost(0, X(0), U(0), 0.0).error(atGoal),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, cost.terminalCost(X(0), 0.0).error(atGoal),
                       1e-12);
}

// Checks that the cost is the same for every step and time.
TEST(StagQuadraticCost, SameForEveryStep) {
  const Values values = testValues();
  EXPECT_DOUBLES_EQUAL(cost.runningCost(0, X(0), U(0), 0.0).error(values),
                       cost.runningCost(7, X(0), U(0), 2.5).error(values),
                       0.0);
  EXPECT_DOUBLES_EQUAL(cost.terminalCost(X(0), 0.0).error(values),
                       cost.terminalCost(X(0), 2.5).error(values), 0.0);
}

// Checks that a zero entry in a diagonal weight ignores that component: with
// no weight on the heading, a pose that differs from the goal only in heading
// costs nothing.
TEST(StagQuadraticCost, ZeroWeightIgnoresComponent) {
  const Matrix3 noHeading{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};
  const Cost positionOnly(goal, noHeading, R, noHeading);
  Values turned;
  turned.insert(X(0), Pose2(goal.x(), goal.y(), goal.theta() + 1.0));
  turned.insert(U(0), Vector2{0.0, 0.0});
  EXPECT_DOUBLES_EQUAL(
      0.0, positionOnly.runningCost(0, X(0), U(0), 0.0).error(turned), 1e-12);
  EXPECT_DOUBLES_EQUAL(
      0.0, positionOnly.terminalCost(X(0), 0.0).error(turned), 1e-12);
}

}  // namespace pose2_quadratic_cost
/* ************************************************************************* */

/* ************************************************************************* */
namespace vector_quadratic_cost {

// Checks the cost for a vector state, where Local(goal, x) = x - goal.
TEST(StagQuadraticCost, VectorState) {
  const Vector4 goal{1.0, 2.0, 0.0, 0.0};
  const Matrix4 Q = Vector4{1.0, 2.0, 0.5, 0.5}.asDiagonal();
  const Matrix2 R = Matrix2::Identity();
  const Matrix4 Qf = 10.0 * Matrix4::Identity();
  const StagQuadraticCost<Vector4, Vector2> cost(goal, Q, R, Qf);

  const Vector4 x{0.5, 1.0, 0.2, -0.1};
  const Vector2 u{0.3, 0.4};
  Values values;
  values.insert(X(0), x);
  values.insert(U(0), u);
  const Vector4 e = x - goal;
  EXPECT_DOUBLES_EQUAL(0.5 * e.dot(Q * e) + 0.5 * u.dot(R * u),
                       cost.runningCost(0, X(0), U(0), 0.0).error(values),
                       1e-9);
  EXPECT_DOUBLES_EQUAL(0.5 * e.dot(Qf * e),
                       cost.terminalCost(X(0), 0.0).error(values), 1e-9);
}

}  // namespace vector_quadratic_cost
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
