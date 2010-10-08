/**
 * @file    testSymbolicBayesNet.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

//#define GTSAM_MAGIC_KEY

#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace std;
using namespace gtsam;

static const varid_t _L_ = 0;
static const varid_t _A_ = 1;
static const varid_t _B_ = 2;
static const varid_t _C_ = 3;

Conditional::shared_ptr
	B(new Conditional(_B_)),
	L(new Conditional(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, equals )
{
	BayesNet<Conditional> f1;
	f1.push_back(B);
	f1.push_back(L);
	BayesNet<Conditional> f2;
	f2.push_back(L);
	f2.push_back(B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, pop_front )
{
	Conditional::shared_ptr
		A(new Conditional(_A_,_B_,_C_)),
		B(new Conditional(_B_,_C_)),
		C(new Conditional(_C_));

	// Expected after pop_front
	BayesNet<Conditional> expected;
	expected.push_back(B);
	expected.push_back(C);

	// Actual
	BayesNet<Conditional> actual;
	actual.push_back(A);
	actual.push_back(B);
	actual.push_back(C);
	actual.pop_front();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, combine )
{
	Conditional::shared_ptr
		A(new Conditional(_A_,_B_,_C_)),
		B(new Conditional(_B_,_C_)),
		C(new Conditional(_C_));

	// p(A|BC)
	BayesNet<Conditional> p_ABC;
	p_ABC.push_back(A);

	// P(BC)=P(B|C)P(C)
	BayesNet<Conditional> p_BC;
	p_BC.push_back(B);
	p_BC.push_back(C);

	// P(ABC) = P(A|BC) P(BC)
	p_ABC.push_back(p_BC);

	BayesNet<Conditional> expected;
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
