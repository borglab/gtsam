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

#include <boost/make_shared.hpp>

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>

using namespace std;
using namespace gtsam;

static const Key _L_ = 0;
static const Key _A_ = 1;
static const Key _B_ = 2;
static const Key _C_ = 3;
static const Key _D_ = 4;
static const Key _E_ = 5;

static SymbolicConditionalUnordered::shared_ptr
  B(new SymbolicConditionalUnordered(_B_)),
  L(new SymbolicConditionalUnordered(_L_, _B_));

/* ************************************************************************* */
TEST( SymbolicBayesNet, equals )
{
  SymbolicBayesNetUnordered f1;
  f1.push_back(B);
  f1.push_back(L);
  SymbolicBayesNetUnordered f2;
  f2.push_back(L);
  f2.push_back(B);
  CHECK(f1.equals(f1));
  CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( SymbolicBayesNet, combine )
{
  SymbolicConditionalUnordered::shared_ptr
    A(new SymbolicConditionalUnordered(_A_,_B_,_C_)),
    B(new SymbolicConditionalUnordered(_B_,_C_)),
    C(new SymbolicConditionalUnordered(_C_));

  // p(A|BC)
  SymbolicBayesNetUnordered p_ABC;
  p_ABC.push_back(A);

  // P(BC)=P(B|C)P(C)
  SymbolicBayesNetUnordered p_BC;
  p_BC.push_back(B);
  p_BC.push_back(C);

  // P(ABC) = P(A|BC) P(BC)
  p_ABC.push_back(p_BC);

  SymbolicBayesNetUnordered expected;
  expected.push_back(A);
  expected.push_back(B);
  expected.push_back(C);

  CHECK(assert_equal(expected,p_ABC));
}

/* ************************************************************************* */
TEST(SymbolicBayesNet, saveGraph) {
  SymbolicBayesNetUnordered bn;
  bn.add(SymbolicConditionalUnordered(_A_, _B_));
  std::vector<Index> keys;
  keys.push_back(_B_);
  keys.push_back(_C_);
  keys.push_back(_D_);
  bn.add(SymbolicConditionalUnordered::FromKeys(keys,2));
  bn.add(SymbolicConditionalUnordered(_D_));

  bn.saveGraph("SymbolicBayesNet.dot");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
