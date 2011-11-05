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

#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/LieVector.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LieVector)
GTSAM_CONCEPT_LIE_INST(LieVector)

/* ************************************************************************* */
TEST( testLieVector, construction ) {
	Vector v = Vector_(3, 1.0, 2.0, 3.0);
	LieVector lie1(v), lie2(v);

	EXPECT(lie1.dim() == 3);
	EXPECT(assert_equal(v, lie1.vector()));
	EXPECT(assert_equal(lie1, lie2));
}

/* ************************************************************************* */
TEST( testLieVector, other_constructors ) {
	Vector init = Vector_(2, 10.0, 20.0);
	LieVector exp(init);
	LieVector a(2,10.0,20.0);
	double data[] = {10,20};
	LieVector b(2,data);
	LieVector c(2.3), c_exp(LieVector(1, 2.3));
	EXPECT(assert_equal(exp, a));
	EXPECT(assert_equal(exp, b));
	EXPECT(assert_equal(b, a));
	EXPECT(assert_equal(c_exp, c));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


