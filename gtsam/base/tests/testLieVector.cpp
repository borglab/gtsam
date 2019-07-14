/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLieVector.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/deprecated/LieVector.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LieVector)
GTSAM_CONCEPT_LIE_INST(LieVector)

//******************************************************************************
TEST(LieVector , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<LieVector>));
  BOOST_CONCEPT_ASSERT((IsManifold<LieVector>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<LieVector>));
}

//******************************************************************************
TEST(LieVector , Invariants) {
  Vector v = Vector3(1.0, 2.0, 3.0);
  LieVector lie1(v), lie2(v);
  check_manifold_invariants(lie1, lie2);
}

//******************************************************************************
TEST( testLieVector, construction ) {
  Vector v = Vector3(1.0, 2.0, 3.0);
  LieVector lie1(v), lie2(v);

  EXPECT(lie1.dim() == 3);
  EXPECT(assert_equal(v, lie1.vector()));
  EXPECT(assert_equal(lie1, lie2));
}

//******************************************************************************
TEST( testLieVector, other_constructors ) {
  Vector init = Vector2(10.0, 20.0);
  LieVector exp(init);
  double data[] = { 10, 20 };
  LieVector b(2, data);
  EXPECT(assert_equal(exp, b));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

