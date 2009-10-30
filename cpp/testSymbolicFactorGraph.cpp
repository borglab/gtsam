/**
 * @file    testSymbolicBayesChain.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "smallExample.h"
#include "FactorGraph-inl.h"
#include "SymbolicFactorGraph.h"
#include "BayesChain-inl.h"
#include "SymbolicConditional.h"
#include "SymbolicBayesChain.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, symbolicFactorGraph )
{
	// construct expected symbolic graph
	SymbolicFactorGraph expected;

	list<string> f1_keys; f1_keys.push_back("x1");
	SymbolicFactor::shared_ptr f1(new SymbolicFactor(f1_keys));
	expected.push_back(f1);

	list<string> f2_keys; f2_keys.push_back("x1"); f2_keys.push_back("x2");
	SymbolicFactor::shared_ptr f2(new SymbolicFactor(f2_keys));
	expected.push_back(f2);

	list<string> f3_keys; f3_keys.push_back("l1"); f3_keys.push_back("x1");
	SymbolicFactor::shared_ptr f3(new SymbolicFactor(f3_keys));
	expected.push_back(f3);

	list<string> f4_keys; f4_keys.push_back("l1"); f4_keys.push_back("x2");
	SymbolicFactor::shared_ptr f4(new SymbolicFactor(f4_keys));
	expected.push_back(f4);

	// construct it from the factor graph graph
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph actual(factorGraph);

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, findAndRemoveFactors )
{
	// construct it from the factor graph graph
	LinearFactorGraph factorGraph = createLinearFactorGraph();
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
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// ask for all factor indices connected to x1
	list<int> x1_factors = fg.factors("x1");
	int x1_indices[] = { 0, 1, 2 };
	list<int> x1_expected(x1_indices, x1_indices + 3);
	CHECK(x1_factors==x1_expected);

	// ask for all factor indices connected to x2
	list<int> x2_factors = fg.factors("x2");
	int x2_indices[] = { 1, 3 };
	list<int> x2_expected(x2_indices, x2_indices + 2);
	CHECK(x2_factors==x2_expected);
}

/* ************************************************************************* */
TEST( LinearFactorGraph, removeAndCombineFactors )
{
	// create a test graph
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

  // combine all factors connected to x1
  SymbolicFactor::shared_ptr actual = fg.removeAndCombineFactors("x1");

	list<string> keys; keys.push_back("l1"); keys.push_back("x1"); keys.push_back("x2");
  SymbolicFactor expected(keys);

  // check if the two factors are the same
  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, eliminateOne )
{
	// create a test graph
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// eliminate
	SymbolicConditional::shared_ptr actual =
  		fg.eliminateOne<SymbolicConditional>("x1");

  // create expected symbolic Conditional
  SymbolicConditional expected("l1","x2");

  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
TEST( LinearFactorGraph, eliminate )
{
  // create expected Chordal bayes Net
  SymbolicConditional::shared_ptr c1(new SymbolicConditional());
  SymbolicConditional::shared_ptr c2(new SymbolicConditional("x1"));
  SymbolicConditional::shared_ptr c3(new SymbolicConditional("l1", "x1"));

  SymbolicBayesChain expected;
  expected.insert("x1", c1);
  expected.insert("l1", c2);
  expected.insert("x2", c3);

  // create a test graph
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
	Ordering ordering;
  ordering.push_back("x2");
  ordering.push_back("l1");
  ordering.push_back("x1");
  SymbolicBayesChain::shared_ptr actual = fg.eliminate(ordering);

  CHECK(assert_equal(expected,*actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
