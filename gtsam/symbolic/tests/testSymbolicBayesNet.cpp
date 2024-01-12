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

#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicConditional.h>

#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;

static const Key _L_ = 0;
static const Key _A_ = 1;
static const Key _B_ = 2;
static const Key _C_ = 3;

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
TEST(SymbolicBayesNet, Dot) {
  using symbol_shorthand::A;
  using symbol_shorthand::X;
  SymbolicBayesNet bn;
  bn.emplace_shared<SymbolicConditional>(X(3), X(2), A(2));
  bn.emplace_shared<SymbolicConditional>(X(2), X(1), A(1));
  bn.emplace_shared<SymbolicConditional>(X(1));

  DotWriter writer;
  writer.positionHints.emplace('a', 2);
  writer.positionHints.emplace('x', 1);
  writer.boxes.emplace(A(1));
  writer.boxes.emplace(A(2));
  
  auto position = writer.variablePos(A(1));
  CHECK(position);
  EXPECT(assert_equal(Vector2(1, 2), *position, 1e-5));

  string actual = bn.dot(DefaultKeyFormatter, writer);
  bn.saveGraph("bn.dot", DefaultKeyFormatter, writer);
  EXPECT(actual ==
         "digraph {\n"
         "  size=\"5,5\";\n"
         "\n"
         "  var6989586621679009793[label=\"a1\", pos=\"1,2!\", shape=box];\n"
         "  var6989586621679009794[label=\"a2\", pos=\"2,2!\", shape=box];\n"
         "  var8646911284551352321[label=\"x1\", pos=\"1,1!\"];\n"
         "  var8646911284551352322[label=\"x2\", pos=\"2,1!\"];\n"
         "  var8646911284551352323[label=\"x3\", pos=\"3,1!\"];\n"
         "\n"
         "  var8646911284551352321->var8646911284551352322\n"
         "  var6989586621679009793->var8646911284551352322\n"
         "  var8646911284551352322->var8646911284551352323\n"
         "  var6989586621679009794->var8646911284551352323\n"
         "}");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
