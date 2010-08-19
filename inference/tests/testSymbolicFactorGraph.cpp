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
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/SymbolicBayesNet.h>
#include <gtsam/inference/FactorGraph-inl.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, eliminate2 )
{
  // create a test graph
	SymbolicFactorGraph fg;
	fg.push_factor("x1", "x2");

	fg.eliminateOne("x1");
	SymbolicFactorGraph expected;
	expected.push_back(boost::shared_ptr<SymbolicFactor>());
	expected.push_factor("x2");

	CHECK(assert_equal(expected, fg));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, constructFromBayesNet )
{
	// create expected factor graph
	SymbolicFactorGraph expected;
	expected.push_factor("l1","x1","x2");
	expected.push_factor("x1","l1");
	expected.push_factor("x1");

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

  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, push_back )
{
	// Create two factor graphs and expected combined graph
	SymbolicFactorGraph fg1, fg2, expected;

	fg1.push_factor("x1");
	fg1.push_factor("x1","x2");

	fg2.push_factor("l1","x1");
	fg2.push_factor("l1","x2");

	expected.push_factor("x1");
	expected.push_factor("x1","x2");
	expected.push_factor("l1","x1");
	expected.push_factor("l1","x2");

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
