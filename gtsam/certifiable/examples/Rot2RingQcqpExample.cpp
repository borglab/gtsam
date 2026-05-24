/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2RingQcqpExample.cpp
 * @brief Minimal Rot2 ring SLAM construction ending at a QcqpProblem.
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <iostream>

using namespace gtsam;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

/**
 * Build the small Rot2 ring used by the QCQP tests. This intentionally uses the
 * existing Frobenius factor conversion path, then stops once QcqpProblem exists.
 */
NonlinearFactorGraph MakeRot2RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }
  return graph;
}

/**
 * Create feasible QCQP matrix values for reporting the constructed problem.
 * SDP construction should consume the QcqpProblem itself, not this graph.
 */
Values MakeRot2RingQcqpValues(size_t numPoses, double delta) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 1>(Symbol('x', i), Rot2::fromAngle(i * delta),
                             &values);
  }
  return values;
}

}  // namespace

int main() {
  constexpr size_t numPoses = 5;
  const double delta = 2.0 * kPi / static_cast<double>(numPoses);

  const NonlinearFactorGraph graph = MakeRot2RingGraph(numPoses, delta);
  const QcqpProblem problem(graph);
  const Values qcqpValues = MakeRot2RingQcqpValues(numPoses, delta);

  std::cout << "Constructed Rot2 ring QCQP" << std::endl;
  std::cout << "poses: " << numPoses << std::endl;
  std::cout << "cost factors: " << problem.costs().size() << std::endl;
  std::cout << "equality constraints: " << problem.eConstraints().size()
            << std::endl;
  std::cout << "cost at feasible ring values: "
            << problem.costs().error(qcqpValues) << std::endl;
  std::cout << "equality violation at feasible ring values: "
            << problem.eConstraints().violationNorm(qcqpValues) << std::endl;
  return 0;
}
