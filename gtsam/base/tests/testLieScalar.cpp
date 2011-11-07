/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLieScalar.cpp
 * @author Kai Ni
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/LieScalar.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LieScalar)
GTSAM_CONCEPT_LIE_INST(LieScalar)

const double tol=1e-9;

/* ************************************************************************* */
TEST( testLieScalar, construction ) {
	double d = 2.;
	LieScalar lie1(d), lie2(d);

	EXPECT_DOUBLES_EQUAL(2., lie1.value(),tol);
	EXPECT_DOUBLES_EQUAL(2., lie2.value(),tol);
	EXPECT(lie1.dim() == 1);
	EXPECT(assert_equal(lie1, lie2));
}

/* ************************************************************************* */
TEST( testLieScalar, localCoordinates ) {
	LieScalar lie1(1.), lie2(3.);

	EXPECT(assert_equal(Vector_(1, 2.), lie1.localCoordinates(lie2)));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
