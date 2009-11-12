/**
 * @file    testInference.cpp
 * @brief   Unit tests for functionality declared in inference.h
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "smallExample.h"
#include "inference-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// The tests below test the *generic* inference algorithms. Some of these have
// specialized versions in the derived classes LinearFactorGraph etc...
/* ************************************************************************* */

/* ************************************************************************* */
TEST(LinearFactorGraph, createSmoother)
{
	LinearFactorGraph fg2 = createSmoother(3);
	LONGS_EQUAL(5,fg2.size());

	// eliminate
	Ordering ordering;
	GaussianBayesNet bayesNet = fg2.eliminate(ordering);
	bayesNet.print("bayesNet");
	FactorGraph<LinearFactor> p_x3 = marginalize<LinearFactor,ConditionalGaussian>(bayesNet, Ordering("x3"));
	FactorGraph<LinearFactor> p_x1 = marginalize<LinearFactor,ConditionalGaussian>(bayesNet, Ordering("x1"));
	CHECK(assert_equal(p_x1,p_x3)); // should be the same because of symmetry
}

/* ************************************************************************* */
TEST( Inference, marginals )
{
	// create and marginalize a small Bayes net on "x"
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  Ordering keys("x");
  FactorGraph<LinearFactor> fg = marginalize<LinearFactor, ConditionalGaussian>(cbn,keys);

  // turn into Bayes net to test easily
  BayesNet<ConditionalGaussian> actual = eliminate<LinearFactor,ConditionalGaussian>(fg,keys);

  // expected is just scalar Gaussian on x
  GaussianBayesNet expected = scalarGaussian("x",4,sqrt(2));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
