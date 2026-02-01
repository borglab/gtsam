/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteLookupDAG.cpp
 *
 *  @date January, 2022
 *  @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteLookupDAG.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteLookupDAG, argmax) {
  using ADT = AlgebraicDecisionTree<Key>;

  // Declare 2 keys
  DiscreteKey A(0, 2), B(1, 2);

  // Create lookup table corresponding to "marginalIsNotMPE" in testDFG.
  DiscreteLookupDAG dag;

  ADT adtB(DiscreteKeys{B, A}, std::vector<double>{0.5, 1. / 3, 0.5, 2. / 3});
  dag.add(1, DiscreteKeys{B, A}, adtB);

  ADT adtA(A, 0.5 * 10 / 19, (2. / 3) * (9. / 19));
  dag.add(1, DiscreteKeys{A}, adtA);

  // The expected MPE is A=1, B=1
  DiscreteValues mpe{{0, 1}, {1, 1}};

  // check:
  auto actualMPE = dag.argmax();
  EXPECT(assert_equal(mpe, actualMPE));
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
