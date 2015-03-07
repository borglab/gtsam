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

#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/LieMatrix.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LieMatrix)
GTSAM_CONCEPT_LIE_INST(LieMatrix)

/* ************************************************************************* */
TEST( LieMatrix, construction ) {
	Matrix m = Matrix_(2,2, 1.0, 2.0, 3.0, 4.0);
	LieMatrix lie1(m), lie2(m);

	EXPECT(lie1.dim() == 4);
	EXPECT(assert_equal(m, lie1.matrix()));
	EXPECT(assert_equal(lie1, lie2));
}

/* ************************************************************************* */
TEST( LieMatrix, other_constructors ) {
	Matrix init = Matrix_(2,2, 10.0, 20.0, 30.0, 40.0);
	LieMatrix exp(init);
	LieMatrix a(2,2,10.0,20.0,30.0,40.0);
	double data[] = {10,30,20,40};
	LieMatrix b(2,2,data);
	EXPECT(assert_equal(exp, a));
	EXPECT(assert_equal(exp, b));
	EXPECT(assert_equal(b, a));
}

/* ************************************************************************* */
TEST(LieMatrix, retract) {
  LieMatrix init(2,2, 1.0,2.0,3.0,4.0);
  Vector update = Vector_(4, 3.0, 4.0, 6.0, 7.0);

  LieMatrix expected(2,2, 4.0, 6.0, 9.0, 11.0);
  LieMatrix actual = init.retract(update);

  EXPECT(assert_equal(expected, actual));

  LieMatrix expectedUpdate = update;
  LieMatrix actualUpdate = init.localCoordinates(actual);

  EXPECT(assert_equal(expectedUpdate, actualUpdate));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


