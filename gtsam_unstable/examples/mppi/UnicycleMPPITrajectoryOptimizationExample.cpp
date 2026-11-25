/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    UnicycleMPPITrajectoryOptimizationExample.cpp
 * @brief   Trajectory optimization of a unicycle with MPPI on a factor graph
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

/**
 * Example of trajectory optimization with MPPI (model predictive path
 * integral control) on a factor graph, for a robot that cannot move sideways:
 *  - A unicycle starts at the origin facing along x and should reach (3, 2)
 *    facing along y after 3 seconds.
 *  - Its state is a Pose2 (x, y, theta) and its control is [v, omega], the
 *    forward speed and the yaw rate.
 *  - The plan is a factor graph over the poses x_0 ... x_H and the controls
 *    u_0 ... u_{H-1}: dynamics factors link each pose to the next, and priors
 *    give a quadratic cost on the poses (toward the goal) and on the controls
 *    (toward zero).
 *  - MPPI samples control sequences around the current plan, rolls each one
 *    out through the dynamics, scores it with the error of the graph, and
 *    moves the plan to the weighted mean of the samples, where cheaper
 *    samples weigh more.
 *  - Here MPPI is a trajectory optimizer: one call to optimize() plans the
 *    whole horizon from x_0, open loop. In MPC the same call runs in a
 *    receding-horizon loop: at each control step x_0 is set to the current
 *    state, the controls are warm-started with the previous plan shifted by
 *    one step, optimize() runs one or a few iterations, and only u_0 is
 *    applied.
 */

// The unicycle model, x_{k+1} = f(x_k, u_k, dt), and its dynamics factor.
#include <gtsam_unstable/planning/dynamics/models/Unicycle.h>

// The planner rolls out controls sampled by a Gaussian proposal.
#include <gtsam/planning/ControlProposals.h>
#include <gtsam/planning/SamplingAlgorithms.h>
#include <gtsam/planning/SamplingPlanner.h>

// The cost terms are priors: a prior with information matrix W on a pose x
// costs 1/2 ||Local(x, mean)||^2_W, where Local(x, mean) is the pose error in
// the tangent space at x.
#include <gtsam/nonlinear/PriorFactor.h>

// The plan is a nonlinear factor graph, and its variables are held in Values.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Symbols name the variables: X(k) is the pose x_k and U(k) the control u_k.
#include <gtsam/inference/Symbol.h>

#include <cmath>
#include <cstdio>
#include <memory>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

int main() {
  // The unicycle, stepped every dt = 0.1 s, over a horizon of H = 30 steps.
  const double dt = 0.1;
  const size_t H = 30;
  const auto model = std::make_shared<const Unicycle>(dt);

  const Pose2 start(0.0, 0.0, 0.0);       // at the origin, facing along x
  const Pose2 goal(3.0, 2.0, M_PI / 2.0);  // at (3, 2), facing along y

  // The cost weights, given as the precisions of the priors' noise models.
  const auto stateCost =
      noiseModel::Diagonal::Precisions(Vector3{1.0, 1.0, 0.1});
  const auto terminalCost =
      noiseModel::Diagonal::Precisions(Vector3{100.0, 100.0, 10.0});
  const auto controlCost = noiseModel::Isotropic::Precision(2, 0.1);

  // MPPI rolls every sample out through the dynamics factors, so they always
  // hold; a constrained noise model marks them as hard constraints.
  const auto dynamicsNoise = noiseModel::Constrained::All(3);

  // The graph: for each step k, the dynamics from x_k to x_{k+1}, a cost on
  // u_k, and a cost on x_{k+1}, with the terminal weights on x_H.
  NonlinearFactorGraph graph;
  KeyVector stateKeys{X(0)}, controlKeys;
  for (size_t k = 0; k < H; ++k) {
    graph.emplace_shared<UnicycleDynamicsFactor>(X(k), U(k), X(k + 1), model,
                                                 dynamicsNoise);
    graph.addPrior(U(k), Vector2{0.0, 0.0}, controlCost);
    graph.addPrior(X(k + 1), goal, k + 1 < H ? stateCost : terminalCost);
    stateKeys.push_back(X(k + 1));
    controlKeys.push_back(U(k));
  }

  // The initial values: x_0 is the pose every rollout starts from, and the
  // controls are the initial plan, here zero. With zero controls the unicycle
  // stays at the start, so these values are the rollout of that plan, and the
  // error of the graph is its cost.
  Values initial;
  initial.insert(X(0), start);
  for (size_t k = 0; k < H; ++k) {
    initial.insert(U(k), Vector2{0.0, 0.0});
    initial.insert(X(k + 1), start);
  }
  std::printf("Initial plan (zero controls): cost %.3f\n",
              graph.error(initial));

  // The proposal draws 1024 control sequences with standard deviation 0.5.
  SamplingPlanner<Pose2, Vector2> planner(stateKeys, controlKeys, dt);
  GaussianProposal proposal(0.25 * Matrix2::Identity(), 1024);
  const MPPI mppi(1.0);
  const auto transition = discreteTransition(
      [&model](const Pose2& pose, const Vector2& control) {
        return model->predict(pose, control);
      });
  const size_t iterations = 30;
  const Values plan =
      planner.optimize(graph, initial, transition, proposal, mppi, iterations);
  std::printf("MPPI plan after %zu iterations: cost %.3f\n\n",
              iterations, graph.error(plan));

  // Print the plan every 0.5 s.
  std::printf("%6s %7s %7s %7s %7s %7s\n", "t [s]", "x", "y", "theta", "v",
              "omega");
  for (size_t k = 0; k < H; k += 5) {
    const Pose2 x = plan.at<Pose2>(X(k));
    const Vector2 u = plan.at<Vector2>(U(k));
    std::printf("%6.1f %7.3f %7.3f %7.3f %7.3f %7.3f\n", k * dt, x.x(), x.y(),
                x.theta(), u(0), u(1));
  }

  const Pose2 last = plan.at<Pose2>(X(H));
  std::printf("\nFinal pose at t = %.1f s: (%.3f, %.3f, %.3f)\n", H * dt,
              last.x(), last.y(), last.theta());
  std::printf("Distance to the goal: %.3f m, heading error: %.3f rad\n",
              (last.translation() - goal.translation()).norm(),
              std::abs(last.theta() - goal.theta()));
  return 0;
}
