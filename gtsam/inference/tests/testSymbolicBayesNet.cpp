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

#include <gtsam/base/Testable.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace std;
using namespace gtsam;

static const Index _L_ = 0;
static const Index _A_ = 1;
static const Index _B_ = 2;
static const Index _C_ = 3;
static const Index _D_ = 4;
static const Index _E_ = 5;

static IndexConditional::shared_ptr
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
TEST(SymbolicBayesNet, find) {
  SymbolicBayesNet bn;
  bn += IndexConditional::shared_ptr(new IndexConditional(_A_, _B_));
  std::vector<Index> keys;
  keys.push_back(_B_);
  keys.push_back(_C_);
  keys.push_back(_D_);
  bn += IndexConditional::shared_ptr(new IndexConditional(keys,2));
  bn += IndexConditional::shared_ptr(new IndexConditional(_D_));

  SymbolicBayesNet::iterator expected = bn.begin();  ++ expected;
  SymbolicBayesNet::iterator actual = bn.find(_C_);
  EXPECT(assert_equal(**expected, **actual));
}

/* ************************************************************************* */
TEST_UNSAFE(SymbolicBayesNet, popLeaf) {
  IndexConditional::shared_ptr
    A(new IndexConditional(_A_,_E_)),
    B(new IndexConditional(_B_,_E_)),
    C(new IndexConditional(_C_,_D_)),
    D(new IndexConditional(_D_,_E_)),
    E(new IndexConditional(_E_));

  // BayesNet after popping A
  SymbolicBayesNet expected1;
  expected1 += B, C, D, E;

  // BayesNet after popping C
  SymbolicBayesNet expected2;
  expected2 += A, B, D, E;

  // BayesNet after popping C and D
  SymbolicBayesNet expected3;
  expected3 += A, B, E;

  // BayesNet after popping C and A
  SymbolicBayesNet expected4;
  expected4 += B, D, E;


  // BayesNet after popping A
  SymbolicBayesNet actual1;
  actual1 += A, B, C, D, E;
  actual1.popLeaf(actual1.find(_A_));

  // BayesNet after popping C
  SymbolicBayesNet actual2;
  actual2 += A, B, C, D, E;
  actual2.popLeaf(actual2.find(_C_));

  // BayesNet after popping C and D
  SymbolicBayesNet actual3;
  actual3 += A, B, C, D, E;
  actual3.popLeaf(actual3.find(_C_));
  actual3.popLeaf(actual3.find(_D_));

  // BayesNet after popping C and A
  SymbolicBayesNet actual4;
  actual4 += A, B, C, D, E;
  actual4.popLeaf(actual4.find(_C_));
  actual4.popLeaf(actual4.find(_A_));

  EXPECT(assert_equal(expected1, actual1));
  EXPECT(assert_equal(expected2, actual2));
  EXPECT(assert_equal(expected3, actual3));
  EXPECT(assert_equal(expected4, actual4));

  // Try to remove a non-leaf node (this test is not working in non-debug mode)
//#undef NDEBUG_SAVED
//#ifdef NDEBUG
//#define NDEBUG_SAVED
//#endif
//
//#undef NDEBUG
//  SymbolicBayesNet actual5;
//  actual5 += A, B, C, D, E;
//  CHECK_EXCEPTION(actual5.popLeaf(actual5.find(_D_)), std::invalid_argument);
//
//#ifdef NDEBUG_SAVED
//#define NDEBUG
//#endif
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
