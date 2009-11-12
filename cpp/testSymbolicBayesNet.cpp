/**
 * @file    testSymbolicBayesNet.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "smallExample.h"
#include "SymbolicBayesNet.h"
#include "SymbolicFactorGraph.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicBayesNet, constructor )
{
	// Create manually
	SymbolicConditional::shared_ptr
		x2(new SymbolicConditional("x2","l1", "x1")),
		l1(new SymbolicConditional("l1","x1")),
		x1(new SymbolicConditional("x1"));
	SymbolicBayesNet expected;
	expected.push_back(x2);
	expected.push_back(l1);
	expected.push_back(x1);

	// Create from a factor graph
	GaussianFactorGraph factorGraph = createGaussianFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
	Ordering ordering;
	ordering += "x2","l1","x1";
  SymbolicBayesNet actual = fg.eliminate(ordering);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, pop_front )
{
	SymbolicConditional::shared_ptr
		A(new SymbolicConditional("A","B","C")),
		B(new SymbolicConditional("B","C")),
		C(new SymbolicConditional("C"));

	// Expected after pop_front
	SymbolicBayesNet expected;
	expected.push_back(B);
	expected.push_back(C);

	// Actual
	SymbolicBayesNet actual;
	actual.push_back(A);
	actual.push_back(B);
	actual.push_back(C);
	actual.pop_front();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, combine )
{
	SymbolicConditional::shared_ptr
		A(new SymbolicConditional("A","B","C")),
		B(new SymbolicConditional("B","C")),
		C(new SymbolicConditional("C"));

	// p(A|BC)
	SymbolicBayesNet p_ABC;
	p_ABC.push_back(A);

	// P(BC)=P(B|C)P(C)
	SymbolicBayesNet p_BC;
	p_BC.push_back(B);
	p_BC.push_back(C);

	// P(ABC) = P(A|BC) P(BC)
	p_ABC.push_back(p_BC);

	SymbolicBayesNet expected;
	expected.push_back(A);
	expected.push_back(B);
	expected.push_back(C);

  CHECK(assert_equal(expected,p_ABC));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
