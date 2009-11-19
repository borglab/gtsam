/**
 * @file    testSymbolicFactorGraph.cpp
 * @brief   Unit tests for a symbolic Factor Graph
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "smallExample.h"
#include "SymbolicFactorGraph.h"
#include "SymbolicBayesNet.h"
#include "FactorGraph-inl.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, symbolicFactorGraph )
{
	// construct expected symbolic graph
	SymbolicFactorGraph expected;

	list<string> f1_keys; f1_keys += "x1";
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
TEST( SymbolicFactorGraph, removeAndCombineFactors )
{
	// create a test graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

  // combine all factors connected to x1
  SymbolicFactor::shared_ptr actual = removeAndCombineFactors(fg,"x1");

	list<string> keys; keys.push_back("l1"); keys.push_back("x1"); keys.push_back("x2");
  SymbolicFactor expected(keys);

  // check if the two factors are the same
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
TEST( SymbolicFactorGraph, constructFromBayesNet )
{
	// create expected factor graph
	FactorGraph<SymbolicFactor> expected;

	list<string> f1_keys; f1_keys += "l1","x1","x2";
	SymbolicFactor::shared_ptr f1(new SymbolicFactor(f1_keys));
	expected.push_back(f1);

	list<string> f2_keys; f2_keys += "x1","l1";
	SymbolicFactor::shared_ptr f2(new SymbolicFactor(f2_keys));
	expected.push_back(f2);

	list<string> f3_keys; f3_keys += "x1";
	SymbolicFactor::shared_ptr f3(new SymbolicFactor(f3_keys));
	expected.push_back(f3);


  // create Bayes Net
  SymbolicConditional::shared_ptr x2(new SymbolicConditional("x2", "l1", "x1"));
  SymbolicConditional::shared_ptr l1(new SymbolicConditional("l1", "x1"));
  SymbolicConditional::shared_ptr x1(new SymbolicConditional("x1"));

  SymbolicBayesNet bayesNet;
  bayesNet.push_back(x2);
  bayesNet.push_back(l1);
  bayesNet.push_back(x1);

  // create actual factor graph from a Bayes Net
	FactorGraph<SymbolicFactor> actual(bayesNet);

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, push_back )
{
	// Create two factor graphs and expected combined graph
	SymbolicFactorGraph fg1, fg2, expected;

	list<string> f1_keys; f1_keys += "x1";
	SymbolicFactor::shared_ptr f1(new SymbolicFactor(f1_keys));
	fg1.push_back(f1);
	expected.push_back(f1);

	list<string> f2_keys; f2_keys.push_back("x1"); f2_keys.push_back("x2");
	SymbolicFactor::shared_ptr f2(new SymbolicFactor(f2_keys));
	fg1.push_back(f2);
	expected.push_back(f2);

	list<string> f3_keys; f3_keys.push_back("l1"); f3_keys.push_back("x1");
	SymbolicFactor::shared_ptr f3(new SymbolicFactor(f3_keys));
	fg2.push_back(f3);
	expected.push_back(f3);

	list<string> f4_keys; f4_keys.push_back("l1"); f4_keys.push_back("x2");
	SymbolicFactor::shared_ptr f4(new SymbolicFactor(f4_keys));
	fg2.push_back(f4);
	expected.push_back(f4);

	// combine
	SymbolicFactorGraph actual = combine(fg1,fg2);
	CHECK(assert_equal(expected, actual));

	// combine using push_back
	fg1.push_back(fg2);
	CHECK(assert_equal(expected, fg1));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
