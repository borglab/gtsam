/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testProfiling.cpp
 * @brief   Profiling hybrid inference
 * @author  Varun Agrawal
 * @date    November 2024
 */

#include <gtsam/discrete/TableDistribution.h>
#include <gtsam/hybrid/HybridGaussianISAM.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// NOTE(Varun): Tests have been disabled for CI since they are long running.
// Please enable them to test performance.

/* ****************************************************************************/
TEST_DISABLED(Profiling, HybridWithPruning) {
  Switching switching(150);
  HybridGaussianISAM incrementalHybrid;
  HybridGaussianFactorGraph graph;

  /***** Run Round 1 *****/
  // std::cout << "{" << std::endl;
  // std::cout << "{ \"round\": 1, \"messages\": [" << std::endl;
  // Add the 3 hybrid factors, x0-x1, x1-x2, x2-x3
  for (size_t i = 0; i < 3; i++) {
    graph.push_back(switching.linearBinaryFactors.at(i));
  }
  // Add the Gaussian factors, 1 prior on x0,
  // 3 measurements on x1, x2, x3
  for (size_t i = 0; i <= 3; i++) {
    graph.push_back(switching.linearUnaryFactors.at(i));
  }

  // Run update with pruning
  size_t maxComponents = 4;
  incrementalHybrid.update(graph, maxComponents);
  // std::cout << "]}" << std::endl;

  for (size_t round = 2; round < 3 /*16*/; round++) {
    // std::cout << "{ \"round\": " << round << ", \"messages\": [" <<
    // std::endl;
    graph = HybridGaussianFactorGraph();
    graph.push_back(switching.linearBinaryFactors.at(round + 1));  // x3-x4
    graph.push_back(switching.linearUnaryFactors.at(round + 2));   // x4

    // Run update with pruning
    incrementalHybrid.update(graph, maxComponents);
    // std::cout << "]}" << std::endl;

    auto discrete_root = incrementalHybrid.roots().at(0);
    discrete_root->conditional()->print();
    for(auto&& key: discrete_root->conditional()->keys()) {
      incrementalHybrid.marginalFactor(key)->print();
    }
    // incrementalHybrid.print();
    tictoc_finishedIteration_();
    tictoc_print_();
    // tictoc_printCsv_(round == 2);
    // std::cout << "resetting" << std::endl;
    // tictoc_reset_();
  }

  // std::cout << "}" << std::endl;
}

/* ****************************************************************************/
// Test approximate inference with an additional pruning step.
TEST_DISABLED(Profiling, HybridNonlinearISAM) {
  Switching switching(15);
  HybridNonlinearISAM incrementalHybrid;
  HybridNonlinearFactorGraph graph;
  Values initial;

  /***** Run Round 1 *****/
  // Add the 3 hybrid factors, x0-x1, x1-x2, x2-x3
  for (size_t i = 0; i < 3; i++) {
    graph.push_back(switching.binaryFactors.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(1), X(2), X(3)
  for (size_t i = 0; i < 4; i++) {
    graph.push_back(switching.unaryFactors.at(i));
    initial.insert<double>(X(i), i + 1);
  }

  // Run update with pruning
  size_t maxComponents = 5;
  incrementalHybrid.update(graph, initial);
  incrementalHybrid.prune(maxComponents);
  HybridGaussianISAM bayesTree = incrementalHybrid.bayesTree();

  bayesTree.saveGraph("BayesTree_Round1.dot");
  incrementalHybrid.saveGraph("HybridISAM_Round1.dot");

  /***** Run Round 2 *****/
  graph = HybridGaussianFactorGraph();
  graph.push_back(switching.binaryFactors.at(3));  // x3-x4
  graph.push_back(switching.unaryFactors.at(4));   // x4 measurement
  initial = Values();
  initial.insert<double>(X(4), 5);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph, initial);
  incrementalHybrid.prune(maxComponents);
  bayesTree = incrementalHybrid.bayesTree();

  bayesTree.saveGraph("BayesTree_Round2.dot");
  incrementalHybrid.saveGraph("HybridISAM_Round2.dot");

  /***** Run Round 3 *****/
  graph = HybridGaussianFactorGraph();
  graph.push_back(switching.binaryFactors.at(4));  // x4-x5
  graph.push_back(switching.unaryFactors.at(5));   // x5 measurement
  initial = Values();
  initial.insert<double>(X(5), 6);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph, initial);
  incrementalHybrid.prune(maxComponents);
  bayesTree = incrementalHybrid.bayesTree();

  bayesTree.saveGraph("BayesTree_Round3.dot");
  incrementalHybrid.saveGraph("HybridISAM_Round3.dot");

  /***** Run Round 4 *****/
  graph = HybridGaussianFactorGraph();
  graph.push_back(switching.binaryFactors.at(5));  // x5-x6
  graph.push_back(switching.unaryFactors.at(6));   // x6 measurement
  initial = Values();
  initial.insert<double>(X(6), 7);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph, initial);
  incrementalHybrid.prune(maxComponents);
  bayesTree = incrementalHybrid.bayesTree();

  bayesTree.saveGraph("BayesTree_Round4.dot");
  incrementalHybrid.saveGraph("HybridISAM_Round4.dot");

  /***** Run Round 5 *****/
  graph = HybridGaussianFactorGraph();
  graph.push_back(switching.binaryFactors.at(6));  // x6-x7
  graph.push_back(switching.unaryFactors.at(7));   // x7 measurement
  initial = Values();
  initial.insert<double>(X(7), 8);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph, initial);
  incrementalHybrid.prune(maxComponents);
  bayesTree = incrementalHybrid.bayesTree();

  bayesTree.saveGraph("BayesTree_Round5.dot");
  incrementalHybrid.saveGraph("HybridISAM_Round5.dot");

  /***** Run Round 6 *****/
  graph = HybridGaussianFactorGraph();
  graph.push_back(switching.binaryFactors.at(7));  // x7-x8
  graph.push_back(switching.unaryFactors.at(8));   // x8 measurement
  initial = Values();
  initial.insert<double>(X(8), 9);

  // Run update with pruning a second time.
  incrementalHybrid.update(graph, initial);
  incrementalHybrid.prune(maxComponents);
  bayesTree = incrementalHybrid.bayesTree();

  bayesTree.saveGraph("BayesTree_Round6.dot");
  incrementalHybrid.saveGraph("HybridISAM_Round6.dot");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
