/**
 * @file    testSymbolicBayesNet.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/Ordering.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/inference/SymbolicBayesNet.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace std;
using namespace gtsam;
using namespace example;

Symbol _B_('B', 0), _L_('L', 0);
SymbolicConditional::shared_ptr
	B(new SymbolicConditional(_B_)),
	L(new SymbolicConditional(_L_, _B_));

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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
