/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInference.cpp
 * @brief   Unit tests for functionality declared in inference.h
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/slam/smallExample.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* */
// The tests below test the *generic* inference algorithms. Some of these have
// specialized versions in the derived classes GaussianFactorGraph etc...
/* ************************************************************************* */

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother)
{
	GaussianFactorGraph fg2;
	Ordering ordering;
	boost::tie(fg2,ordering) = createSmoother(3);
	LONGS_EQUAL(5,fg2.size());

	// eliminate
	vector<Index> x3var; x3var.push_back(ordering["x3"]);
	vector<Index> x1var; x1var.push_back(ordering["x1"]);
	GaussianBayesNet p_x3 = *GaussianSequentialSolver(*GaussianSequentialSolver(fg2).jointFactorGraph(x3var)).eliminate();
	GaussianBayesNet p_x1 = *GaussianSequentialSolver(*GaussianSequentialSolver(fg2).jointFactorGraph(x1var)).eliminate();
	CHECK(assert_equal(*p_x1.back(),*p_x3.front())); // should be the same because of symmetry
}

/* ************************************************************************* */
TEST( Inference, marginals )
{
	// create and marginalize a small Bayes net on "x"
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  vector<Index> xvar; xvar.push_back(0);
  GaussianBayesNet actual = *GaussianSequentialSolver(*GaussianSequentialSolver(GaussianFactorGraph(cbn)).jointFactorGraph(xvar)).eliminate();

  // expected is just scalar Gaussian on x
  GaussianBayesNet expected = scalarGaussian(0, 4, sqrt(2));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
