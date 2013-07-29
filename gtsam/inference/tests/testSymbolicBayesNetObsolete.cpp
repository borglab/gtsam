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
#include <gtsam/inference/IndexConditionalOrdered.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>

using namespace std;
using namespace gtsam;

static const Index _L_ = 0;
static const Index _A_ = 1;
static const Index _B_ = 2;
static const Index _C_ = 3;
static const Index _D_ = 4;
static const Index _E_ = 5;

static IndexConditionalOrdered::shared_ptr
  B(new IndexConditionalOrdered(_B_)),
  L(new IndexConditionalOrdered(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNetOrdered, equals )
{
  BayesNetOrdered<IndexConditionalOrdered> f1;
  f1.push_back(B);
  f1.push_back(L);
  BayesNetOrdered<IndexConditionalOrdered> f2;
  f2.push_back(L);
  f2.push_back(B);
  CHECK(f1.equals(f1));
  CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( SymbolicBayesNetOrdered, pop_front )
{
  IndexConditionalOrdered::shared_ptr
    A(new IndexConditionalOrdered(_A_,_B_,_C_)),
    B(new IndexConditionalOrdered(_B_,_C_)),
    C(new IndexConditionalOrdered(_C_));

  // Expected after pop_front
  BayesNetOrdered<IndexConditionalOrdered> expected;
  expected.push_back(B);
  expected.push_back(C);

  // Actual
  BayesNetOrdered<IndexConditionalOrdered> actual;
  actual.push_back(A);
  actual.push_back(B);
  actual.push_back(C);
  actual.pop_front();

  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicBayesNetOrdered, combine )
{
  IndexConditionalOrdered::shared_ptr
    A(new IndexConditionalOrdered(_A_,_B_,_C_)),
    B(new IndexConditionalOrdered(_B_,_C_)),
    C(new IndexConditionalOrdered(_C_));

  // p(A|BC)
  BayesNetOrdered<IndexConditionalOrdered> p_ABC;
  p_ABC.push_back(A);

  // P(BC)=P(B|C)P(C)
  BayesNetOrdered<IndexConditionalOrdered> p_BC;
  p_BC.push_back(B);
  p_BC.push_back(C);

  // P(ABC) = P(A|BC) P(BC)
  p_ABC.push_back(p_BC);

  BayesNetOrdered<IndexConditionalOrdered> expected;
  expected.push_back(A);
  expected.push_back(B);
  expected.push_back(C);

  CHECK(assert_equal(expected,p_ABC));
}

/* ************************************************************************* */
TEST(SymbolicBayesNetOrdered, find) {
  SymbolicBayesNetOrdered bn;
  bn += IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(_A_, _B_));
  std::vector<Index> keys;
  keys.push_back(_B_);
  keys.push_back(_C_);
  keys.push_back(_D_);
  bn += IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(keys,2));
  bn += IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(_D_));

  SymbolicBayesNetOrdered::iterator expected = bn.begin();  ++ expected;
  SymbolicBayesNetOrdered::iterator actual = bn.find(_C_);
  EXPECT(assert_equal(**expected, **actual));
}

/* ************************************************************************* */
TEST_UNSAFE(SymbolicBayesNetOrdered, popLeaf) {
  IndexConditionalOrdered::shared_ptr
    A(new IndexConditionalOrdered(_A_,_E_)),
    B(new IndexConditionalOrdered(_B_,_E_)),
    C(new IndexConditionalOrdered(_C_,_D_)),
    D(new IndexConditionalOrdered(_D_,_E_)),
    E(new IndexConditionalOrdered(_E_));

  // BayesNet after popping A
  SymbolicBayesNetOrdered expected1;
  expected1 += B, C, D, E;

  // BayesNet after popping C
  SymbolicBayesNetOrdered expected2;
  expected2 += A, B, D, E;

  // BayesNet after popping C and D
  SymbolicBayesNetOrdered expected3;
  expected3 += A, B, E;

  // BayesNet after popping C and A
  SymbolicBayesNetOrdered expected4;
  expected4 += B, D, E;


  // BayesNet after popping A
  SymbolicBayesNetOrdered actual1;
  actual1 += A, B, C, D, E;
  actual1.popLeaf(actual1.find(_A_));

  // BayesNet after popping C
  SymbolicBayesNetOrdered actual2;
  actual2 += A, B, C, D, E;
  actual2.popLeaf(actual2.find(_C_));

  // BayesNet after popping C and D
  SymbolicBayesNetOrdered actual3;
  actual3 += A, B, C, D, E;
  actual3.popLeaf(actual3.find(_C_));
  actual3.popLeaf(actual3.find(_D_));

  // BayesNet after popping C and A
  SymbolicBayesNetOrdered actual4;
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
//  SymbolicBayesNetOrdered actual5;
//  actual5 += A, B, C, D, E;
//  CHECK_EXCEPTION(actual5.popLeaf(actual5.find(_D_)), std::invalid_argument);
//
//#ifdef NDEBUG_SAVED
//#define NDEBUG
//#endif
}

/* ************************************************************************* */
TEST(SymbolicBayesNetOrdered, saveGraph) {
  SymbolicBayesNetOrdered bn;
  bn += IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(_A_, _B_));
  std::vector<Index> keys;
  keys.push_back(_B_);
  keys.push_back(_C_);
  keys.push_back(_D_);
  bn += IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(keys,2));
  bn += IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(_D_));

  bn.saveGraph("SymbolicBayesNet.dot");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
