/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLieMatrix.cpp
 * @author Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/deprecated/LieMatrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LieMatrix)
GTSAM_CONCEPT_LIE_INST(LieMatrix)

/* ************************************************************************* */
TEST( LieMatrix, construction ) {
  Matrix m = (Matrix(2,2) << 1.0,2.0, 3.0,4.0).finished();
  LieMatrix lie1(m), lie2(m);

  EXPECT(traits<LieMatrix>::GetDimension(m) == 4);
  EXPECT(assert_equal(m, lie1.matrix()));
  EXPECT(assert_equal(lie1, lie2));
}

/* ************************************************************************* */
TEST( LieMatrix, other_constructors ) {
  Matrix init = (Matrix(2,2) << 10.0,20.0, 30.0,40.0).finished();
  LieMatrix exp(init);
  double data[] = {10,30,20,40};
  LieMatrix b(2,2,data);
  EXPECT(assert_equal(exp, b));
}

/* ************************************************************************* */
TEST(LieMatrix, retract) {
  LieMatrix init((Matrix(2,2) << 1.0,2.0,3.0,4.0).finished());
  Vector update = (Vector(4) << 3.0, 4.0, 6.0, 7.0).finished();

  LieMatrix expected((Matrix(2,2) << 4.0, 6.0, 9.0, 11.0).finished());
  LieMatrix actual = traits<LieMatrix>::Retract(init,update);

  EXPECT(assert_equal(expected, actual));

  Vector expectedUpdate = update;
  Vector actualUpdate = traits<LieMatrix>::Local(init,actual);

  EXPECT(assert_equal(expectedUpdate, actualUpdate));

  Vector expectedLogmap = (Vector(4) << 1, 2, 3, 4).finished();
  Vector actualLogmap = traits<LieMatrix>::Logmap(LieMatrix((Matrix(2,2) << 1.0, 2.0, 3.0, 4.0).finished()));
  EXPECT(assert_equal(expectedLogmap, actualLogmap));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


