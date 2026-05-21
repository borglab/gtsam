/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseMinimal.cpp
 * @brief   Minimal end-to-end staircase usage on a 5-pose Rot2 ring with
 *          random initial rotations.
 * @author  Zhexin Xu  (xu.zhex@northeastern.edu)
 * @author  David M. Rosen
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <iostream>
#include <random>

using namespace gtsam;

int main() {
  constexpr size_t N = 5;
  const double delta = 2.0 * M_PI / N;

  // Ring x_{i+1} = x_i * Rot2(2π/N), no measurement noise.
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < N; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % N), Rot2::fromAngle(delta));
  }

  Values initial;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni(-M_PI, M_PI);
  for (size_t i = 0; i < N; ++i) {
    InsertQcqpValue<Rot2, 2>(Symbol('x', i), Rot2::fromAngle(uni(rng)),
                             &initial);
  }

  QcqpProblem qcqp(graph, /*K=*/2);
  RiemannianStaircaseOptimizer rso(qcqp, initial);
  const auto result = rso.optimize<Rot2>();

  std::cout << "certified : " << (result.certified ? "yes" : "no") << "\n"
            << "final rank: " << result.finalRank << "\n"
            << "cost      : " << result.costPerLevel.back() << "\n";
  for (size_t i = 0; i < N; ++i) {
    const Rot2 R = result.rounded.at<Rot2>(Symbol('x', i));
    std::cout << "  x" << i << " = " << R.theta() << " rad\n";
  }
  return result.certified ? 0 : 1;
}
