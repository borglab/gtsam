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
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicConditional.h>

using namespace std;
using namespace gtsam;

static const Key _L_ = 0;
static const Key _A_ = 1;
static const Key _B_ = 2;
static const Key _C_ = 3;
static const Key _D_ = 4;

static SymbolicConditional::shared_ptr
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
TEST( SymbolicBayesNet, combine )
{
  SymbolicConditional::shared_ptr
    A(new SymbolicConditional(_A_,_B_,_C_)),
    B(new SymbolicConditional(_B_,_C_)),
    C(new SymbolicConditional(_C_));

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
TEST(SymbolicBayesNet, saveGraph) {
  SymbolicBayesNet bn;
  bn += SymbolicConditional(_A_, _B_);
  KeyVector keys {_B_, _C_, _D_};
  bn += SymbolicConditional::FromKeys(keys,2);
  bn += SymbolicConditional(_D_);

  bn.saveGraph("SymbolicBayesNet.dot");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
