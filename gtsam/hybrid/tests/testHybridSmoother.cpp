/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridSmoother.cpp
 * @brief   Unit tests for HybridSmoother
 * @author  Varun Agrawal
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/hybrid/HybridSmoother.h>
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

#include <string>

#include "Switching.h"

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;
using symbol_shorthand::Z;

namespace estimation_fixture {
std::vector<double> measurements = {0, 1, 2, 2, 2, 2,  3,  4,  5,  6, 6,
                                    7, 8, 9, 9, 9, 10, 11, 11, 11, 11};
// Ground truth discrete seq
std::vector<size_t> discrete_seq = {1, 1, 0, 0, 0, 1, 1, 1, 1, 0,
                                    1, 1, 1, 0, 0, 1, 1, 0, 0, 0};

Switching InitializeEstimationProblem(
    const size_t K, const double between_sigma, const double measurement_sigma,
    const std::vector<double>& measurements,
    const std::string& transitionProbabilityTable,
    HybridNonlinearFactorGraph* graph, Values* initial) {
  Switching switching(K, between_sigma, measurement_sigma, measurements,
                      transitionProbabilityTable);

  // Add prior on M(0)
  graph->push_back(switching.modeChain.at(0));

  // Add the X(0) prior
  graph->push_back(switching.unaryFactors.at(0));
  initial->insert(X(0), switching.linearizationPoint.at<double>(X(0)));

  return switching;
}

}  // namespace estimation_fixture

/****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(HybridSmoother, IncrementalSmoother) {
  using namespace estimation_fixture;

  size_t K = 5;

  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  HybridNonlinearFactorGraph graph;
  Values initial;
  Switching switching = InitializeEstimationProblem(
      K, 1.0, 0.1, measurements, "1/1 1/1", &graph, &initial);

  HybridSmoother smoother;
  constexpr size_t maxNrLeaves = 5;

  // Loop over timesteps from 1...K-1
  for (size_t k = 1; k < K; k++) {
    if (k > 1) graph.push_back(switching.modeChain.at(k - 1));  // Mode chain
    graph.push_back(switching.binaryFactors.at(k - 1));         // Motion Model
    graph.push_back(switching.unaryFactors.at(k));              // Measurement

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    smoother.update(graph, initial, maxNrLeaves);

    // Clear all the factors from the graph
    graph.resize(0);
  }

  auto& hybridBayesNet = smoother.hybridBayesNet();
#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(11, hybridBayesNet.at(5)->asDiscrete()->nrValues());
#else
  EXPECT_LONGS_EQUAL(16, hybridBayesNet.at(5)->asDiscrete()->nrValues());
#endif

  // Get the continuous delta update as well as
  // the optimal discrete assignment.
  HybridValues delta = hybridBayesNet.optimize();

  // Check discrete assignment
  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }
  EXPECT(assert_equal(expected_discrete, delta.discrete()));

  // Update nonlinear solution and verify
  Values result = initial.retract(delta.continuous());
  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/****************************************************************************/
// Test if pruned Bayes net is set to correct error and no errors are thrown.
TEST(HybridSmoother, ValidPruningError) {
  using namespace estimation_fixture;

  size_t K = 8;

  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  HybridNonlinearFactorGraph graph;
  Values initial;
  Switching switching = InitializeEstimationProblem(
      K, 0.1, 0.1, measurements, "1/1 1/1", &graph, &initial);
  HybridSmoother smoother;

  constexpr size_t maxNrLeaves = 3;
  for (size_t k = 1; k < K; k++) {
    if (k > 1) graph.push_back(switching.modeChain.at(k - 1));  // Mode chain
    graph.push_back(switching.binaryFactors.at(k - 1));         // Motion Model
    graph.push_back(switching.unaryFactors.at(k));              // Measurement

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    smoother.update(graph, initial, maxNrLeaves);

    // Clear all the factors from the graph
    graph.resize(0);
  }

  auto& hybridBayesNet = smoother.hybridBayesNet();
#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(14, hybridBayesNet.at(8)->asDiscrete()->nrValues());
#else
  EXPECT_LONGS_EQUAL(128, hybridBayesNet.at(8)->asDiscrete()->nrValues());
#endif

  // Get the continuous delta update as well as
  // the optimal discrete assignment.
  HybridValues delta = smoother.hybridBayesNet().optimize();

  auto errorTree = smoother.hybridBayesNet().errorTree(delta.continuous());
  EXPECT_DOUBLES_EQUAL(1e-8, errorTree(delta.discrete()), 1e-8);
}

/****************************************************************************/
// Test if dead mode removal works.
TEST(HybridSmoother, DeadModeRemoval) {
  using namespace estimation_fixture;

  size_t K = 8;

  // Switching example of robot moving in 1D
  // with given measurements and equal mode priors.
  HybridNonlinearFactorGraph graph;
  Values initial;
  Switching switching = InitializeEstimationProblem(
      K, 0.1, 0.1, measurements, "1/1 1/1", &graph, &initial);

  // Smoother with dead mode removal enabled.
  HybridSmoother smoother(true);

  constexpr size_t maxNrLeaves = 3;
  for (size_t k = 1; k < K; k++) {
    if (k > 1) graph.push_back(switching.modeChain.at(k - 1));  // Mode chain
    graph.push_back(switching.binaryFactors.at(k - 1));         // Motion Model
    graph.push_back(switching.unaryFactors.at(k));              // Measurement

    initial.insert(X(k), switching.linearizationPoint.at<double>(X(k)));

    smoother.update(graph, initial, maxNrLeaves);

    // Clear all the factors from the graph
    graph.resize(0);
  }

  // Get the continuous delta update as well as
  // the optimal discrete assignment.
  HybridValues delta = smoother.hybridBayesNet().optimize();

  // Check discrete assignment
  DiscreteValues expected_discrete;
  for (size_t k = 0; k < K - 1; k++) {
    expected_discrete[M(k)] = discrete_seq[k];
  }
  EXPECT(assert_equal(expected_discrete, delta.discrete()));

  // Update nonlinear solution and verify
  Values result = initial.retract(delta.continuous());
  Values expected_continuous;
  for (size_t k = 0; k < K; k++) {
    expected_continuous.insert(X(k), measurements[k]);
  }
  EXPECT(assert_equal(expected_continuous, result));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
