/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianJunctionTree.cpp
 * @brief   Unit tests for GaussianJunctionTree
 * @author  Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>

#include "linearTestProblems.h"

/* ************************************************************************* */
TEST(GaussianJunctionTree, eliminateInPlace)
{
  // Copy the example graph because we'll be modifying it in-place

  GaussianFactorGraph graph = simpleGraph2.clone();

  Ordering ordering = Ordering::COLAMD(graph);

  // Eliminate the original graph, keeping two copies of the junction tree
  GaussianJunctionTree gjt(GaussianEliminationTree(graph, ordering));
  GaussianBayesTree::shared_ptr bayesTree = gjt.eliminate(EliminateQR).first;
  GaussianBayesTree::shared_ptr bayesTreeCopy = gjt.eliminate(EliminateQR).first;
  EXPECT(assert_equal(*bayesTree, *bayesTreeCopy));

  // Test re-elimination without modifications - no change should happen
  gjt.eliminateInPlace(*bayesTree, EliminateQR);
  EXPECT(assert_equal(*bayesTreeCopy, *bayesTree));

  // Modify the original graph
  JacobianFactor& factor7 = dynamic_cast<JacobianFactor&>(*graph[7]);
  factor7.getA(factor7.begin()) *= 2.0; // Modify matrix in-place

  // Get the new Bayes Tree, not in-place
  GaussianBayesTree::shared_ptr newBayesTree = gjt.eliminate(EliminateQR).first;

  // The new Bayes Tree should be different than the old one
  EXPECT(!bayesTreeCopy->equals(*newBayesTree, 1e-2));

  // Now re-eliminate the Gaussian Junction Tree with the modified factor
  gjt.eliminateInPlace(*bayesTree, EliminateQR);

  // The re-eliminated Bayes tree should now be equal to the new one
  EXPECT(assert_equal(*newBayesTree, *bayesTree));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */


