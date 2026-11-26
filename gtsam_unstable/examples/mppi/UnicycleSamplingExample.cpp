// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file    UnicycleSamplingExample.cpp
 * @brief   Compare three sampling algorithms on one unicycle planning problem.
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/planning/SamplingPlanner.h>

#include <iostream>
#include <string>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

int main(int argc, char** argv) {
  const std::string choice = argc > 1 ? argv[1] : "all";
  if (argc > 2 || (choice != "all" && choice != "cem" && choice != "mppi" &&
                   choice != "predictive")) {
    std::cerr << "Usage: UnicycleSamplingExample [all|cem|mppi|predictive]\n";
    return 1;
  }
  const size_t horizon = 20, samples = 256, iterations = 12;
  const double dt = 0.1;
  const Pose2 goal(1.5, 0.75, 0.8);
  const Vector2 zero = Vector2::Zero();
  const auto effort = noiseModel::Isotropic::Sigma(2, 4.0);
  const auto terminal = noiseModel::Diagonal::Sigmas(Vector3{0.1, 0.1, 0.2});

  NonlinearFactorGraph cost;
  Values initial;
  initial.insert(X(0), Pose2());
  KeyVector states{X(0)}, controls;
  for (size_t k = 0; k < horizon; ++k) {
    states.push_back(X(k + 1));
    controls.push_back(U(k));
    initial.insert(U(k), zero);
    cost.addPrior(U(k), zero, effort);
  }
  cost.addPrior(X(horizon), goal, terminal);

  // Body-frame tangent velocity at the current pose.
  const auto field = [](const Pose2&, const Vector2& u) {
    return Vector3{u[0], 0.0, u[1]};
  };
  const SamplingPlanner<Pose2, Vector2> planner(states, controls, dt);
  std::cout << "Initial cost: "
            << cost.error(
                   planner.rollout(initial, Matrix::Zero(2, horizon), field))
            << '\n';
  const auto run = [&](const char* name, const auto& algorithm,
                       const Matrix2& covariance) {
    GaussianProposal proposal(covariance, samples);
    const Values plan =
        planner.optimize(cost, initial, field, proposal, algorithm, iterations);
    const Pose2 pose = plan.at<Pose2>(states.back());
    std::cout << name << " cost: " << cost.error(plan)
              << ", final pose: " << pose << '\n';
  };
  // MPPI uses separate exploration scales for speed and yaw rate.
  const Matrix2 mppiCovariance =
      Vector2{0.346, 0.243}.array().square().matrix().asDiagonal();
  if (choice == "cem" || choice == "all")
    run("CEM", CEM(32), Matrix2::Identity());
  if (choice == "mppi" || choice == "all")
    run("MPPI", MPPI(1.0), mppiCovariance);
  if (choice == "predictive" || choice == "all")
    run("Predictive sampling", PredictiveSampling(), Matrix2::Identity());
}
