/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridBayesTree.cpp
 * @brief   Unit tests for HybridBayesTree
 * @author  Varun Agrawal
 * @date    August 2022
 */

#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianISAM.h>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************/
// Test multifrontal optimize
TEST(HybridBayesTree, OptimizeMultifrontal) {
  Switching s(4);

  HybridBayesTree::shared_ptr hybridBayesTree =
      s.linearizedFactorGraph.eliminateMultifrontal();
  HybridValues delta = hybridBayesTree->optimize();

  VectorValues expectedValues;
  expectedValues.insert(X(0), -0.999904 * Vector1::Ones());
  expectedValues.insert(X(1), -0.99029 * Vector1::Ones());
  expectedValues.insert(X(2), -1.00971 * Vector1::Ones());
  expectedValues.insert(X(3), -1.0001 * Vector1::Ones());

  EXPECT(assert_equal(expectedValues, delta.continuous(), 1e-5));
}

/* ****************************************************************************/
// Test for optimizing a HybridBayesTree with a given assignment.
TEST(HybridBayesTree, OptimizeAssignment) {
  Switching s(4);

  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph1;

  // Add the 3 hybrid factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 1; i < 4; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(1),
  // 3 measurements on X(2), X(3), X(4)
  graph1.push_back(s.linearizedFactorGraph.at(0));
  for (size_t i = 4; i <= 7; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the discrete factors
  for (size_t i = 7; i <= 9; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  isam.update(graph1);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;

  VectorValues delta = isam.optimize(assignment);

  // The linearization point has the same value as the key index,
  // e.g. X(1) = 1, X(2) = 2,
  // but the factors specify X(k) = k-1, so delta should be -1.
  VectorValues expected_delta;
  expected_delta.insert(make_pair(X(0), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(1), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(2), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(3), -Vector1::Ones()));

  EXPECT(assert_equal(expected_delta, delta));

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < s.K; k++) ordering.push_back(X(k));

  const auto [hybridBayesNet, remainingFactorGraph] =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  GaussianBayesNet gbn = hybridBayesNet->choose(assignment);
  VectorValues expected = gbn.optimize();

  EXPECT(assert_equal(expected, delta));
}

/* ****************************************************************************/
// Test for optimizing a HybridBayesTree.
TEST(HybridBayesTree, Optimize) {
  Switching s(4);

  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph1;

  // Add the 3 hybrid factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 1; i < 4; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(2), X(3), X(4)
  graph1.push_back(s.linearizedFactorGraph.at(0));
  for (size_t i = 4; i <= 6; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the discrete factors
  for (size_t i = 7; i <= 9; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  isam.update(graph1);

  HybridValues delta = isam.optimize();

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < s.K; k++) ordering.push_back(X(k));

  const auto [hybridBayesNet, remainingFactorGraph] =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  DiscreteFactorGraph dfg;
  for (auto&& f : *remainingFactorGraph) {
    auto discreteFactor = dynamic_pointer_cast<DiscreteFactor>(f);
    assert(discreteFactor);
    dfg.push_back(discreteFactor);
  }

  // Add the probabilities for each branch
  DiscreteKeys discrete_keys = {{M(0), 2}, {M(1), 2}, {M(2), 2}};
  vector<double> probs = {0.012519475, 0.041280228, 0.075018647, 0.081663656,
                          0.037152205, 0.12248971,  0.07349729,  0.08};
  dfg.emplace_shared<DecisionTreeFactor>(discrete_keys, probs);

  DiscreteValues expectedMPE = dfg.optimize();
  VectorValues expectedValues = hybridBayesNet->optimize(expectedMPE);

  EXPECT(assert_equal(expectedMPE, delta.discrete()));
  EXPECT(assert_equal(expectedValues, delta.continuous()));
}

/* ****************************************************************************/
// Test for choosing a GaussianBayesTree from a HybridBayesTree.
TEST(HybridBayesTree, Choose) {
  Switching s(4);

  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph1;

  // Add the 3 hybrid factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 1; i < 4; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(2), X(3), X(4)
  graph1.push_back(s.linearizedFactorGraph.at(0));
  for (size_t i = 4; i <= 6; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the discrete factors
  for (size_t i = 7; i <= 9; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  isam.update(graph1);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;

  GaussianBayesTree gbt = isam.choose(assignment);

  // Specify ordering so it matches that of HybridGaussianISAM.
  Ordering ordering(KeyVector{X(0), X(1), X(2), X(3), M(0), M(1), M(2)});
  auto bayesTree = s.linearizedFactorGraph.eliminateMultifrontal(ordering);

  auto expected_gbt = bayesTree->choose(assignment);

  EXPECT(assert_equal(expected_gbt, gbt));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
