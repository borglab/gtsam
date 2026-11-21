/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    UnicycleMPPIMPCExample.cpp
 * @brief   Model predictive control of a unicycle with MPPI on a factor graph
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 */

/**
 * Example of model predictive control (MPC) with MPPI (model predictive path
 * integral control) on a factor graph:
 *  - A unicycle starts at the origin facing along x and should reach (3, 2)
 *    facing along y. The simulation ends when it is within tolerance of the
 *    goal.
 *  - Its state is a Pose2 (x, y, theta) and its control is [v, omega], the
 *    forward speed and the yaw rate.
 *  - The plan is the factor graph of UnicycleMPPITrajectoryOptimizationExample:
 *    dynamics factors over a 3 second horizon, and quadratic costs on the
 *    poses (toward the goal) and on the controls (toward zero).
 *  - MPC plans again at every control step, over a horizon that recedes with
 *    time: x_0 is set to the current pose, the controls are warm-started with
 *    the previous plan shifted by one step, MPPI runs one iteration, and only
 *    the first control u_0 is applied.
 *  - The simulated unicycle follows the same model with process noise, so it
 *    drifts from the plan, and each new plan starts from the pose it reached.
 */

// The unicycle model, x_{k+1} = f(x_k, u_k, dt), and its dynamics factor.
#include <gtsam_unstable/mpc_stag/dynamics/models/Unicycle.h>

// The vanilla MPPI controller, which samples Gaussian control noise.
#include <gtsam_unstable/mpc_stag/controller/VanillaMPPIController.h>

// The cost terms are priors: a prior with information matrix W on a pose x
// costs 1/2 ||Local(x, mean)||^2_W, where Local(x, mean) is the pose error in
// the tangent space at x.
#include <gtsam/nonlinear/PriorFactor.h>

// The plan is a nonlinear factor graph, and its variables are held in Values.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Symbols name the variables: X(k) is the pose x_k and U(k) the control u_k.
#include <gtsam/inference/Symbol.h>

// The process noise of the simulated unicycle is drawn with a Sampler.
#include <gtsam/linear/Sampler.h>

#include <cmath>
#include <cstdio>
#include <memory>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

int main() {
  // The unicycle, stepped every dt = 0.1 s; each plan looks H = 30 steps
  // ahead.
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

  // The graph is built once: its keys and costs are the same at every control
  // step, and only x_0 and the initial controls change. For each step k, it
  // holds the dynamics from x_k to x_{k+1}, a cost on u_k, and a cost on
  // x_{k+1}, with the terminal weights on x_H.
  NonlinearFactorGraph graph;
  KeyVector controlKeys;
  for (size_t k = 0; k < H; ++k) {
    graph.emplace_shared<UnicycleDynamicsFactor>(X(k), U(k), X(k + 1), model,
                                                 dynamicsNoise);
    graph.addPrior(U(k), Vector2{0.0, 0.0}, controlCost);
    graph.addPrior(X(k + 1), goal, k + 1 < H ? stateCost : terminalCost);
    controlKeys.push_back(U(k));
  }

  // The values of the first plan: the start pose and zero controls. The other
  // poses only need a value of the right type.
  Values values;
  values.insert(X(0), start);
  for (size_t k = 0; k < H; ++k) {
    values.insert(U(k), Vector2{0.0, 0.0});
    values.insert(X(k + 1), start);
  }

  // The MPPI parameters: as in UnicycleMPPITrajectoryOptimizationExample, but
  // with one iteration per control step, since the warm start carries the
  // plan from one step to the next.
  VanillaMPPIParams params;
  params.numSamples = 1024;
  params.iterations = 1;
  params.controlCovariance = 0.25 * Matrix2::Identity();
  VanillaMPPIController<Pose2, Vector2> controller(params);

  // The process noise of the simulated unicycle: standard deviations of
  // 0.01 m, 0.01 m and 0.01 rad per step, in the unicycle's own frame.
  const Sampler processNoise(Vector3{0.01, 0.01, 0.01});

  // The simulation ends when the unicycle is within 0.05 m and 0.1 rad of the
  // goal, or after 10 s.
  const double positionTolerance = 0.05;
  const double headingTolerance = 0.1;
  const size_t maxSteps = 100;
  const auto distanceToGoal = [&goal](const Pose2& pose) {
    return (pose.translation() - goal.translation()).norm();
  };
  const auto headingError = [&goal](const Pose2& pose) {
    return std::abs(goal.rotation().between(pose.rotation()).theta());
  };

  // Run the MPC loop, and print the pose and the applied control every 0.5 s.
  Pose2 pose = start;
  size_t step = 0;
  std::printf("%6s %7s %7s %7s %7s %7s\n", "t [s]", "x", "y", "theta", "v",
              "omega");
  for (; step < maxSteps; ++step) {
    if (distanceToGoal(pose) < positionTolerance &&
        headingError(pose) < headingTolerance) {
      break;
    }

    // Plan from the current pose.
    values.update(X(0), pose);
    const Values plan = controller.optimize(graph, values, controlKeys);

    // Apply the first control to the simulated unicycle.
    const Vector2 u = plan.at<Vector2>(U(0));
    if (step % 5 == 0) {
      std::printf("%6.1f %7.3f %7.3f %7.3f %7.3f %7.3f\n", step * dt, pose.x(),
                  pose.y(), pose.theta(), u(0), u(1));
    }
    pose = model->predict(pose, u).retract(processNoise.sample());

    // Warm start the next plan: shift the controls by one step, and start the
    // last one at zero.
    values = plan;
    for (size_t k = 0; k + 1 < H; ++k) {
      values.update(U(k), plan.at<Vector2>(U(k + 1)));
    }
    values.update(U(H - 1), Vector2{0.0, 0.0});
  }

  std::printf("\n%s at t = %.1f s: pose (%.3f, %.3f, %.3f)\n",
              step < maxSteps ? "Reached the goal"
                              : "Stopped without reaching the goal",
              step * dt, pose.x(), pose.y(), pose.theta());
  std::printf("Distance to the goal: %.4f m, heading error: %.4f rad\n",
              distanceToGoal(pose), headingError(pose));
  return 0;
}
