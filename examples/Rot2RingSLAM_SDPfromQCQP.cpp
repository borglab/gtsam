/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2RingQcqpExample.cpp
 * @brief Minimal example that builds a Rot2 ring graph and converts it to QCQP.
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <cmath>
#include <iostream>

using namespace gtsam;

/**
 * Build the same small rotation-only ring used by the QCQP unit tests.
 *
 * Each edge is a FrobeniusBetweenFactor<Rot2>, which currently supports the
 * `NonlinearFactor::qcqpFactors` conversion hook. The returned graph is still a
 * normal nonlinear factor graph; `QcqpProblem` performs the QCQP extraction.
 */
static NonlinearFactorGraph BuildRot2RingGraph(size_t numPoses) {
  NonlinearFactorGraph graph;
  const double delta = 2.0 * std::acos(-1.0) / static_cast<double>(numPoses);

  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }

  return graph;
}

int main() {
  constexpr size_t kNumPoses = 5;

  const NonlinearFactorGraph graph = BuildRot2RingGraph(kNumPoses);
  const QcqpProblem problem(graph);

  std::cout << "Rot2 ring nonlinear factors: " << graph.size() << std::endl;
  std::cout << "QCQP objective factors: " << problem.costs().size()
            << std::endl;

  return 0;
}
