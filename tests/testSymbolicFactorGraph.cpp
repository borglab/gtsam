/**
 * @file    testSymbolicFactorGraph.cpp
 * @brief   Unit tests for a symbolic Factor Graph
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/Ordering.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/SymbolicBayesNet.h>
#include <gtsam/inference/FactorGraph-inl.h>

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, symbolicFactorGraph )
{
	// construct expected symbolic graph
	SymbolicFactorGraph expected;
	expected.push_factor("x1");
	expected.push_factor("x1","x2");
	expected.push_factor("l1","x1");
	expected.push_factor("l1","x2");

	// construct it from the factor graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph actual(factorGraph);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, findAndRemoveFactors )
{
	// construct it from the factor graph graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph actual(factorGraph);
  SymbolicFactor::shared_ptr f1 = actual[0];
  SymbolicFactor::shared_ptr f3 = actual[2];
	actual.findAndRemoveFactors("x2");

	// construct expected graph after find_factors_and_remove
	SymbolicFactorGraph expected;
	SymbolicFactor::shared_ptr null;
	expected.push_back(f1);
	expected.push_back(null);
	expected.push_back(f3);
	expected.push_back(null);

	CHECK(assert_equal(expected, actual));
}
/* ************************************************************************* */
TEST( SymbolicFactorGraph, factors)
{
	// create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// ask for all factor indices connected to x1
	list<size_t> x1_factors = fg.factors("x1");
	int x1_indices[] = { 0, 1, 2 };
	list<size_t> x1_expected(x1_indices, x1_indices + 3);
	CHECK(x1_factors==x1_expected);

	// ask for all factor indices connected to x2
	list<size_t> x2_factors = fg.factors("x2");
	int x2_indices[] = { 1, 3 };
	list<size_t> x2_expected(x2_indices, x2_indices + 2);
	CHECK(x2_factors==x2_expected);
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, removeAndCombineFactors )
{
	// create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

  // combine all factors connected to x1
  SymbolicFactor::shared_ptr actual = removeAndCombineFactors(fg,"x1");

  // check result
  SymbolicFactor expected("l1","x1","x2");
  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, eliminateOne )
{
	// create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// eliminate
	SymbolicConditional::shared_ptr actual = fg.eliminateOne("x1");

  // create expected symbolic Conditional
  SymbolicConditional expected("x1","l1","x2");

  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminate )
{
  // create expected Chordal bayes Net
  SymbolicConditional::shared_ptr x2(new SymbolicConditional("x2", "l1", "x1"));
  SymbolicConditional::shared_ptr l1(new SymbolicConditional("l1", "x1"));
  SymbolicConditional::shared_ptr x1(new SymbolicConditional("x1"));

  SymbolicBayesNet expected;
  expected.push_back(x2);
  expected.push_back(l1);
  expected.push_back(x1);

  // create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
	Ordering ordering;
	ordering += "x2","l1","x1";
  SymbolicBayesNet actual = fg.eliminate(ordering);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
