/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridEstimation.cpp
 * @brief   Unit tests for end-to-end Hybrid Estimation
 * @author  Varun Agrawal
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/hybrid/HybridSmoother.h>
#include <gtsam/hybrid/MixtureFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
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

Ordering getOrdering(HybridGaussianFactorGraph& factors,
                     const HybridGaussianFactorGraph& newFactors) {
  factors += newFactors;
  // Get all the discrete keys from the factors
  KeySet allDiscrete = factors.discreteKeys();

  // Create KeyVector with continuous keys followed by discrete keys.
  KeyVector newKeysDiscreteLast;
  const KeySet newFactorKeys = newFactors.keys();
  // Insert continuous keys first.
  for (auto& k : newFactorKeys) {
    if (!allDiscrete.exists(k)) {
      newKeysDiscreteLast.push_back(k);
    }
  }

  // Insert discrete keys at the end
  std::copy(allDiscrete.begin(), allDiscrete.end(),
            std::back_inserter(newKeysDiscreteLast));

  const VariableIndex index(factors);

  // Get an ordering where the new keys are eliminated last
  Ordering ordering = Ordering::ColamdConstrainedLast(
      index, KeyVector(newKeysDiscreteLast.begin(), newKeysDiscreteLast.end()),
      true);
  return ordering;
}

/****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST(HybridNonlinearISAM, Incremental) {
  size_t K = 10;
  std::vector<double> measurements = {0, 1, 2, 2, 2, 2, 3, 4, 5, 6, 6};
  // Ground truth discrete seq
  std::vector<size_t> discrete_seq = {1, 1, 0, 0, 0, 1, 1, 1, 1, 0};
  Switching switching(K, 1.0, 0.1, measurements);
  // HybridNonlinearISAM smoother;
  HybridSmoother smoother;
  HybridNonlinearFactorGraph graph;
  Values initial;

  // switching.nonlinearFactorGraph.print();
  // switching.linearizationPoint.print();
  // Add the X(1) prior
  graph.push_back(switching.nonlinearFactorGraph.at(0));
  initial.insert(X(1), switching.linearizationPoint.at<double>(X(1)));

  HybridGaussianFactorGraph linearized;
  HybridGaussianFactorGraph bayesNet;

  for (size_t k = 1; k < K; k++) {
    // Motion Model
    graph.push_back(switching.nonlinearFactorGraph.at(k));
    // Measurement
    graph.push_back(switching.nonlinearFactorGraph.at(k + K - 1));

    initial.insert(X(k + 1), switching.linearizationPoint.at<double>(X(k + 1)));

    // std::cout << "\n=============  " << k << std::endl;
    // graph.print();

    bayesNet = smoother.hybridBayesNet();
    linearized = *graph.linearize(initial);
    Ordering ordering = getOrdering(bayesNet, linearized);

    ordering.print();
    smoother.update(linearized, ordering, 3);
    // if (k == 2) exit(0);
    // smoother.hybridBayesNet().print();
    graph.resize(0);
    // initial.clear();
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
