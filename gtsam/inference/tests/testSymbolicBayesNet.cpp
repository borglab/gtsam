/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicBayesNet.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

//#define GTSAM_MAGIC_KEY

#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace std;
using namespace gtsam;

static const Index _L_ = 0;
static const Index _A_ = 1;
static const Index _B_ = 2;
static const Index _C_ = 3;

IndexConditional::shared_ptr
	B(new IndexConditional(_B_)),
	L(new IndexConditional(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, equals )
{
	BayesNet<IndexConditional> f1;
	f1.push_back(B);
	f1.push_back(L);
	BayesNet<IndexConditional> f2;
	f2.push_back(L);
	f2.push_back(B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, pop_front )
{
	IndexConditional::shared_ptr
		A(new IndexConditional(_A_,_B_,_C_)),
		B(new IndexConditional(_B_,_C_)),
		C(new IndexConditional(_C_));

	// Expected after pop_front
	BayesNet<IndexConditional> expected;
	expected.push_back(B);
	expected.push_back(C);

	// Actual
	BayesNet<IndexConditional> actual;
	actual.push_back(A);
	actual.push_back(B);
	actual.push_back(C);
	actual.pop_front();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, combine )
{
	IndexConditional::shared_ptr
		A(new IndexConditional(_A_,_B_,_C_)),
		B(new IndexConditional(_B_,_C_)),
		C(new IndexConditional(_C_));

	// p(A|BC)
	BayesNet<IndexConditional> p_ABC;
	p_ABC.push_back(A);

	// P(BC)=P(B|C)P(C)
	BayesNet<IndexConditional> p_BC;
	p_BC.push_back(B);
	p_BC.push_back(C);

	// P(ABC) = P(A|BC) P(BC)
	p_ABC.push_back(p_BC);

	BayesNet<IndexConditional> expected;
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
