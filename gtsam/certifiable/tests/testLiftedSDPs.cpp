/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLiftedSDPs.cpp
 * @brief Tests for QCQP-backed lifted SDP objective assembly.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace lifted_sdp_tests {

constexpr double kPi = 3.141592653589793238462643383279502884;

// Build a cycle of relative Rot2 measurements.
NonlinearFactorGraph Rot2RingGraph(size_t numPoses, double delta) {
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < numPoses; ++i) {
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', (i + 1) % numPoses),
        Rot2::fromAngle(delta));
  }
  return graph;
}

// Create lifted QCQP values with a controlled angular perturbation.
Values Rot2RingQcqpValues(size_t numPoses, double delta, double perturbation) {
  Values values;
  for (size_t i = 0; i < numPoses; ++i) {
    InsertQcqpValue<Rot2, 1>(
        Symbol('x', i),
        Rot2::fromAngle(i * delta + perturbation * static_cast<double>(i)),
        &values);
  }
  return values;
}

// Assemble the local rank-one matrix in a Hessian factor's key order.
Matrix BuildLocalX(const HessianFactor& H, const Values& values) {
  std::vector<Vector> localValues;
  DenseIndex totalDim = 0;
  for (auto it = H.begin(); it != H.end(); ++it) {
    Vector x = values.at<Matrix>(*it).col(0);
    if (x.size() != H.getDim(it)) {
      throw std::runtime_error("BuildLocalX: QCQP value dimension mismatch.");
    }
    totalDim += x.size();
    localValues.push_back(std::move(x));
  }

  Matrix X_f = Matrix::Zero(totalDim, totalDim);
  DenseIndex rowStart = 0;
  for (const Vector& x_i : localValues) {
    DenseIndex colStart = 0;
    for (const Vector& x_j : localValues) {
      X_f.block(rowStart, colStart, x_i.size(), x_j.size()) =
          x_i * x_j.transpose();
      colStart += x_j.size();
    }
    rowStart += x_i.size();
  }
  return X_f;
}

// Evaluate the lifted trace objective directly from rank-one matrices.
double ComputeLiftedObjective(const QcqpProblem& problem,
                              const Values& values) {
  double objective = 0.0;
  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error("ComputeLiftedObjective: expected QpCost.");
    }

    const HessianFactor& H = cost->hessianFactor();
    if (H.linearTerm().norm() > 0.0 || H.constantTerm() != 0.0) {
      throw std::runtime_error(
          "ComputeLiftedObjective: linear/constant terms are not supported.");
    }

    const Matrix Q_f = H.information();
    const Matrix X_f = BuildLocalX(H, values);
    objective += 0.5 * Q_f.cwiseProduct(X_f).sum();
  }
  return objective;
}

// Verifies that lifting preserves the QCQP objective at a feasible assignment.
TEST(LiftedSDPs, Rot2_QcqpObjectiveMatchesLiftedObjective) {
  constexpr size_t N = 5;
  const double delta = 2.0 * kPi / static_cast<double>(N);
  constexpr double perturbation = 0.03;

  const NonlinearFactorGraph graph = Rot2RingGraph(N, delta);
  const QcqpProblem problem(graph);
  const Values qcqpValues = Rot2RingQcqpValues(N, delta, perturbation);

  const double qcqpObjective = problem.costs().error(qcqpValues);
  const double sdpObjective = ComputeLiftedObjective(problem, qcqpValues);

  std::cout << "qcqpObjective: " << qcqpObjective << std::endl;
  std::cout << "sdpObjective: " << sdpObjective << std::endl;

  EXPECT_DOUBLES_EQUAL(qcqpObjective, sdpObjective, 1e-12);
}

}  // namespace lifted_sdp_tests
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
