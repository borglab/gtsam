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
 * @author  Frank Dellaert
 */

/**
 * Example of model predictive control (MPC) with MPPI (model predictive path
 * integral control) on a factor graph:
 *  - A unicycle starts at the origin facing along x and should reach (3, 2)
 *    facing along y. The simulation ends when it is within tolerance of the
 *    goal.
 *  - Its state is a Pose2 (x, y, theta) and its control is [v, omega], the
 *    forward speed and the yaw rate.
 *  - The same discrete transition and quadratic costs as the open-loop
 *    example predict poses over a 3 second horizon and score the distance
 *    to the goal and the control effort.
 *  - MPC plans again at every control step, over a horizon that recedes with
 *    time: x_0 is set to the current pose, the controls are warm-started with
 *    the previous plan shifted by one step, MPPI runs one iteration, and only
 *    the first control u_0 is applied.
 *  - The simulated unicycle follows the same model with process noise, so it
 *    drifts from the plan, and each new plan starts from the pose it reached.
 */

// The unicycle model supplies the discrete transition for planning and simulation.
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

// The process noise of the simulated unicycle is drawn with a Sampler.
#include <gtsam/linear/Sampler.h>

#include <cmath>
#include <cstdio>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

int main() {
  // The unicycle, stepped every dt = 0.1 s; each plan looks H = 30 steps
  // ahead.
  const double dt = 0.1;
  const size_t H = 30;
  const Unicycle model(dt);

  const Pose2 start(0.0, 0.0, 0.0);       // at the origin, facing along x
  const Pose2 goal(3.0, 2.0, M_PI / 2.0);  // at (3, 2), facing along y

  // The cost weights, given as the precisions of the priors' noise models.
  const auto stateCost =
      noiseModel::Diagonal::Precisions(Vector3{1.0, 1.0, 0.1});
  const auto terminalCost =
      noiseModel::Diagonal::Precisions(Vector3{100.0, 100.0, 10.0});
  const auto controlCost = noiseModel::Isotropic::Precision(2, 0.1);

  // The transition enforces the dynamics at every step. Only the running
  // and terminal costs need to be evaluated on each new trajectory.
  NonlinearFactorGraph graph;
  KeyVector stateKeys{X(0)}, controlKeys;
  for (size_t k = 0; k < H; ++k) {
    graph.addPrior(U(k), Vector2{0.0, 0.0}, controlCost);
    graph.addPrior(X(k + 1), goal, k + 1 < H ? stateCost : terminalCost);
    stateKeys.push_back(X(k + 1));
    controlKeys.push_back(U(k));
  }

  // The values of the first plan: the start pose and zero controls. The other
  // poses are supplied by the rollout.
  Values values;
  values.insert(X(0), start);
  for (size_t k = 0; k < H; ++k) {
    values.insert(U(k), Vector2{0.0, 0.0});
  }

  // The proposal draws 1024 control sequences with standard deviation 0.5.
  SamplingPlanner<Pose2, Vector2> planner(stateKeys, controlKeys, dt);
  GaussianProposal proposal(0.25 * Matrix2::Identity(), 1024);
  const MPPI mppi(1.0);
  const auto transition = discreteTransition(
      [&model](const Pose2& pose, const Vector2& control) {
        return model.predict(pose, control);
      });

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
    const Values plan =
        planner.optimize(graph, values, transition, proposal, mppi, 1);

    // Apply the first control to the simulated unicycle.
    const Vector2 u = plan.at<Vector2>(U(0));
    if (step % 5 == 0) {
      std::printf("%6.1f %7.3f %7.3f %7.3f %7.3f %7.3f\n", step * dt, pose.x(),
                  pose.y(), pose.theta(), u(0), u(1));
    }
    pose = model.predict(pose, u).retract(processNoise.sample());

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
