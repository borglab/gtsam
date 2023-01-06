/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridPruning.cpp
 * @brief   Unit tests for end-to-end Hybrid Estimation
 * @author  Varun Agrawal
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/hybrid/HybridSmoother.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

#include "Switching.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;

/****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST_DISABLED(HybridPruning, ISAM) {
  size_t K = 16;
  std::vector<double> measurements = {0, 1, 2, 2, 2, 2,  3,  4,  5,  6, 6,
                                      7, 8, 9, 9, 9, 10, 11, 11, 11, 11};
  // Ground truth discrete seq
  std::vector<size_t> discrete_seq = {1, 1, 0, 0, 0, 1, 1, 1, 1, 0,
                                      1, 1, 1, 0, 0, 1, 1, 0, 0, 0};
  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  Switching switching(K, 1.0, 0.1, measurements, "1/1 1/1");
  HybridNonlinearISAM isam;
  HybridNonlinearFactorGraph graph;
  Values initial;

  // Add the X(0) prior
  graph.push_back(switching.nonlinearFactorGraph.at(0));
  initial.insert(X(0), switching.linearizationPoint.at<double>(X(0)));

  HybridGaussianFactorGraph linearized;
  HybridGaussianFactorGraph bayesNet;

  for (size_t k = 1; k < K; k++) {
    // Motion Model
    graph.push_back(switching.nonlinearFactorGraph.at(k));
    // Measurement
    graph.push_back(switching.nonlinearFactorGraph.at(k + K - 1));

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    isam.update(graph, initial, 3);
    graph.resize(0);
    initial.clear();
  }

  Values result  = isam.estimate();
  DiscreteValues assignment = isam.assignment();

  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }

  std::cout << "\n\n\nNonlinear Version!!\n\n" << std::endl;
  GTSAM_PRINT(expected_discrete);
  GTSAM_PRINT(assignment);
  EXPECT(assert_equal(expected_discrete, assignment));

  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(HybridPruning, GaussianISAM) {
  size_t K = 16;
  std::vector<double> measurements = {0, 1, 2, 2, 2, 2,  3,  4,  5,  6, 6,
                                      7, 8, 9, 9, 9, 10, 11, 11, 11, 11};
  // Ground truth discrete seq
  std::vector<size_t> discrete_seq = {1, 1, 0, 0, 0, 1, 1, 1, 1, 0,
                                      1, 1, 1, 0, 0, 1, 1, 0, 0, 0};
  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  Switching switching(K, 1.0, 0.1, measurements, "1/1 1/1");
  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph;
  Values initial;

  // Add the X(0) prior
  graph.push_back(switching.linearizedFactorGraph.at(0));
  initial.insert(X(0), switching.linearizationPoint.at<double>(X(0)));

  HybridGaussianFactorGraph linearized;
  HybridGaussianFactorGraph bayesNet;

  for (size_t k = 1; k < K; k++) {
    // Motion Model
    graph.push_back(switching.linearizedFactorGraph.at(k));
    // Measurement
    graph.push_back(switching.linearizedFactorGraph.at(k + K - 1));

    // initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    isam.update(graph, 3);
    graph.resize(0);
    // initial.clear();
  }

  HybridValues values = isam.optimize();

  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }

  EXPECT(assert_equal(expected_discrete, values.discrete()));

  // Values expected_continuous;
  // for (size_t k = 0; k < K; k++) {
  //   expected_continuous.insert(X(k), measurements[k]);
  // }
  // EXPECT(assert_equal(expected_continuous, result));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
