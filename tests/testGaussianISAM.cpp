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

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/inference/ISAM.h>
#include <gtsam/geometry/Rot2.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
// Some numbers that should be consistent among all smoother tests

static const double tol = 1e-4;

/* ************************************************************************* */
TEST( ISAM, iSAM_smoother )
{
  Ordering ordering;
  for (int t = 1; t <= 7; t++) ordering += X(t);

  // Create smoother with 7 nodes
  GaussianFactorGraph smoother = createSmoother(7, ordering).first;

  // run iSAM for every factor
  GaussianISAM actual;
  BOOST_FOREACH(boost::shared_ptr<GaussianFactor> factor, smoother) {
    GaussianFactorGraph factorGraph;
    factorGraph.push_back(factor);
    actual.update(factorGraph);
  }

  // Create expected Bayes Tree by solving smoother with "natural" ordering
  BayesTree<GaussianConditional>::shared_ptr bayesTree = GaussianMultifrontalSolver(smoother).eliminate();
  GaussianISAM expected(*bayesTree);

  // Verify sigmas in the bayes tree
  BOOST_FOREACH(const GaussianBayesTree::sharedClique& clique, bayesTree->nodes()) {
    GaussianConditional::shared_ptr conditional = clique->conditional();
    size_t dim = conditional->dim();
    EXPECT(assert_equal(gtsam::ones(dim), conditional->get_sigmas(), tol));
  }

  // Check whether BayesTree is correct
  EXPECT(assert_equal(expected, actual));

  // obtain solution
  VectorValues e(VectorValues::Zero(7,2)); // expected solution
  VectorValues optimized = optimize(actual); // actual solution
  EXPECT(assert_equal(e, optimized));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
