/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PointMassMPPITrajectoryOptimizationExample.cpp
 * @brief   Trajectory optimization of a point mass with MPPI on a factor graph
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 */

/**
 * Example of trajectory optimization with MPPI (model predictive path
 * integral control) on a factor graph:
 *  - A point mass in the plane starts at rest at the origin and should come
 *    to rest at the goal (3, 2) after 3 seconds.
 *  - Its state is [px, py, vx, vy] and its control is the acceleration
 *    [ax, ay].
 *  - The plan is a factor graph over the states x_0 ... x_H and the controls
 *    u_0 ... u_{H-1}: dynamics factors link each state to the next, and
 *    priors give a quadratic cost on the states (toward the goal) and on the
 *    controls (toward zero).
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

// The point mass model, x_{k+1} = f(x_k, u_k, dt), and its dynamics factor.
#include <gtsam_unstable/mpc_stag/dynamics/models/PointMass.h>

// The vanilla MPPI controller, which samples Gaussian control noise.
#include <gtsam_unstable/mpc_stag/controller/VanillaMPPIController.h>

// The cost terms are priors: a prior with information matrix W on a variable
// x costs 1/2 ||x - mean||^2_W.
#include <gtsam/nonlinear/PriorFactor.h>

// The plan is a nonlinear factor graph, and its variables are held in Values.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Symbols name the variables: X(k) is the state x_k and U(k) the control u_k.
#include <gtsam/inference/Symbol.h>

#include <cstdio>
#include <memory>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

int main() {
  // The point mass, stepped every dt = 0.1 s, over a horizon of H = 30 steps.
  const double dt = 0.1;
  const size_t H = 30;
  const auto model = std::make_shared<const PointMass>(dt);

  const Vector4 start{0.0, 0.0, 0.0, 0.0};  // at rest at the origin
  const Vector4 goal{3.0, 2.0, 0.0, 0.0};   // at rest at (3, 2)

  // The cost weights, given as the precisions of the priors' noise models.
  const auto stateCost =
      noiseModel::Diagonal::Precisions(Vector4{1.0, 1.0, 0.1, 0.1});
  const auto terminalCost =
      noiseModel::Diagonal::Precisions(Vector4{100.0, 100.0, 10.0, 10.0});
  const auto controlCost = noiseModel::Isotropic::Precision(2, 0.1);

  // MPPI rolls every sample out through the dynamics factors, so they always
  // hold; a constrained noise model marks them as hard constraints.
  const auto dynamicsNoise = noiseModel::Constrained::All(4);

  // The graph: for each step k, the dynamics from x_k to x_{k+1}, a cost on
  // u_k, and a cost on x_{k+1}, with the terminal weights on x_H.
  NonlinearFactorGraph graph;
  KeyVector controlKeys;
  for (size_t k = 0; k < H; ++k) {
    graph.emplace_shared<PointMassDynamicsFactor>(X(k), U(k), X(k + 1), model,
                                                  dynamicsNoise);
    graph.addPrior(U(k), Vector2{0.0, 0.0}, controlCost);
    graph.addPrior(X(k + 1), goal, k + 1 < H ? stateCost : terminalCost);
    controlKeys.push_back(U(k));
  }

  // The initial values: x_0 is the state every rollout starts from, and the
  // controls are the initial plan, here zero. With zero controls the point
  // mass stays at the start, so these values are the rollout of that plan,
  // and the error of the graph is its cost.
  Values initial;
  initial.insert(X(0), start);
  for (size_t k = 0; k < H; ++k) {
    initial.insert(U(k), Vector2{0.0, 0.0});
    initial.insert(X(k + 1), start);
  }
  std::printf("Initial plan (zero controls): cost %.3f\n",
              graph.error(initial));

  // The MPPI parameters: 1024 samples per iteration, 30 iterations, control
  // noise with a standard deviation of 0.5 m/s^2 on each axis, and the
  // default inverse temperature 1.
  VanillaMPPIParams params;
  params.numSamples = 1024;
  params.iterations = 30;
  params.controlCovariance = 0.25 * Matrix2::Identity();

  // Optimize: MPPI returns the planned controls and the states rolled out
  // with them.
  VanillaMPPIController<Vector4, Vector2> controller(params);
  const Values plan = controller.optimize(graph, initial, controlKeys);
  std::printf("MPPI plan after %zu iterations: cost %.3f\n\n",
              params.iterations, graph.error(plan));

  // Print the plan every 0.5 s.
  std::printf("%6s %7s %7s %7s %7s %7s %7s\n", "t [s]", "px", "py", "vx",
              "vy", "ax", "ay");
  for (size_t k = 0; k < H; k += 5) {
    const Vector4 x = plan.at<Vector4>(X(k));
    const Vector2 u = plan.at<Vector2>(U(k));
    std::printf("%6.1f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n", k * dt, x(0),
                x(1), x(2), x(3), u(0), u(1));
  }

  const Vector4 last = plan.at<Vector4>(X(H));
  std::printf("\nFinal state at t = %.1f s: position (%.3f, %.3f), "
              "velocity (%.3f, %.3f)\n",
              H * dt, last(0), last(1), last(2), last(3));
  std::printf("Distance to the goal: %.3f m\n",
              (last.head<2>() - goal.head<2>()).norm());
  return 0;
}
