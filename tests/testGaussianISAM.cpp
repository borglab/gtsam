/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianISAM.cpp
 * @brief   Unit tests for GaussianISAM
 * @author  Michael Kaess
 */

#include <CppUnitLite/TestHarness.h>

#include <tests/smallExample.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/inference/Ordering.h>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( ISAM, iSAM_smoother )
{
  Ordering ordering;
  for (int t = 1; t <= 7; t++) ordering += X(t);

  // Create smoother with 7 nodes
  GaussianFactorGraph smoother = createSmoother(7);

  // run iSAM for every factor
  GaussianISAM actual;
  for(std::shared_ptr<GaussianFactor> factor: smoother) {
    GaussianFactorGraph factorGraph;
    factorGraph.push_back(factor);
    actual.update(factorGraph);
  }

  // Create expected Bayes Tree by solving smoother with "natural" ordering
  GaussianBayesTree expected = *smoother.eliminateMultifrontal(ordering);

  // Verify sigmas in the bayes tree
  for (const auto& [key, clique] : expected.nodes()) {
    GaussianConditional::shared_ptr conditional = clique->conditional();
    EXPECT(!conditional->get_model());
  }

  // Check whether BayesTree is correct
  EXPECT(assert_equal(GaussianFactorGraph(expected).augmentedHessian(), GaussianFactorGraph(actual).augmentedHessian()));

  // obtain solution
  VectorValues e; // expected solution
  for (int t = 1; t <= 7; t++) e.insert(X(t), Vector::Zero(2));
  VectorValues optimized = actual.optimize(); // actual solution
  EXPECT(assert_equal(e, optimized));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
