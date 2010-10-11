/**
 * @file    testSymbolicFactorGraph.cpp
 * @brief   Unit tests for a symbolic Factor Graph
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/inference-inl.h>

using namespace std;
using namespace gtsam;

static const Index vx2=0;
static const Index vx1=1;
static const Index vl1=2;

/* ************************************************************************* */
TEST( SymbolicFactorGraph, eliminate2 )
{
  // create a test graph
	SymbolicFactorGraph fg;
	fg.push_factor(vx2, vx1);

	VariableIndex<> variableIndex(fg);
	Inference::EliminateOne(fg, variableIndex, vx2);
	SymbolicFactorGraph expected;
	expected.push_back(boost::shared_ptr<Factor>());
	expected.push_factor(vx1);

	CHECK(assert_equal(expected, fg));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, constructFromBayesNet )
{
	// create expected factor graph
	SymbolicFactorGraph expected;
	expected.push_factor(vx2,vx1,vl1);
	expected.push_factor(vx1,vl1);
	expected.push_factor(vx1);

  // create Bayes Net
  Conditional::shared_ptr x2(new Conditional(vx2, vx1, vl1));
  Conditional::shared_ptr l1(new Conditional(vx1, vl1));
  Conditional::shared_ptr x1(new Conditional(vx1));

  BayesNet<Conditional> bayesNet;
  bayesNet.push_back(x2);
  bayesNet.push_back(l1);
  bayesNet.push_back(x1);

  // create actual factor graph from a Bayes Net
	SymbolicFactorGraph actual(bayesNet);

  CHECK(assert_equal((SymbolicFactorGraph)expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, push_back )
{
	// Create two factor graphs and expected combined graph
	SymbolicFactorGraph fg1, fg2, expected;

	fg1.push_factor(vx1);
	fg1.push_factor(vx2,vx1);

	fg2.push_factor(vx1,vl1);
	fg2.push_factor(vx2,vl1);

	expected.push_factor(vx1);
	expected.push_factor(vx2,vx1);
	expected.push_factor(vx1,vl1);
	expected.push_factor(vx2,vl1);

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
