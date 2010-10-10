/**
 * @file    testInference.cpp
 * @brief   Unit tests for functionality declared in inference.h
 * @author  Frank Dellaert
 */

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/inference-inl.h>

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
	list<varid_t> x3var; x3var.push_back(ordering["x3"]);
  list<varid_t> x1var; x1var.push_back(ordering["x1"]);
	GaussianBayesNet p_x3 = *Inference::Marginal(fg2, x3var);
	GaussianBayesNet p_x1 = *Inference::Marginal(fg2, x1var);
	CHECK(assert_equal(*p_x1.back(),*p_x3.front())); // should be the same because of symmetry
}

/* ************************************************************************* */
TEST( Inference, marginals )
{
	// create and marginalize a small Bayes net on "x"
  GaussianBayesNet cbn = createSmallGaussianBayesNet();
  list<varid_t> xvar; xvar.push_back(0);
  GaussianBayesNet actual = *Inference::Marginal(GaussianFactorGraph(cbn), xvar);

  // expected is just scalar Gaussian on x
  GaussianBayesNet expected = scalarGaussian(0, 4, sqrt(2));
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
