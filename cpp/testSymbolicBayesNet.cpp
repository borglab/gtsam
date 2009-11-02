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
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph fg(factorGraph);

	// eliminate it
	Ordering ordering;
	ordering += "x2","l1","x1";
  SymbolicBayesNet::shared_ptr actual = fg.eliminate(ordering);

  CHECK(assert_equal(expected, *actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
