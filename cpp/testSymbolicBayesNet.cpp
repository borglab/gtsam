/**
 * @file    testSymbolicBayesNet.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "SymbolicBayesNet.h"
#include "SymbolicFactorGraph.h"

using namespace std;
using namespace gtsam;

Symbol _B_('B', 0), _L_('L', 0);
SymbolicConditional::shared_ptr
	B(new SymbolicConditional(_B_)),
	L(new SymbolicConditional(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, equals )
{
	SymbolicBayesNet f1;
	f1.push_back(B);
	f1.push_back(L);
	SymbolicBayesNet f2;
	f2.push_back(L);
	f2.push_back(B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
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
