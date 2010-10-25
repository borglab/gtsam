/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testFixedVector.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/FixedVector.h>

using namespace gtsam;

typedef FixedVector<5> Vector5;
typedef FixedVector<3> Vector3;

static const double tol = 1e-9;

/* ************************************************************************* */
TEST( testFixedVector, conversions ) {
	double data1[] = {1.0, 2.0, 3.0};
	Vector v1  = Vector_(3, data1);
	Vector3 fv1(v1), fv2(data1);

	Vector actFv2(fv2);
	CHECK(assert_equal(v1, actFv2));
}

/* ************************************************************************* */
TEST( testFixedVector, variable_constructor ) {
	Vector3 act(3, 1.0, 2.0, 3.0);
	DOUBLES_EQUAL(1.0, act(0), tol);
	DOUBLES_EQUAL(2.0, act(1), tol);
	DOUBLES_EQUAL(3.0, act(2), tol);
}

/* ************************************************************************* */
TEST( testFixedVector, equals ) {
	Vector3 vec1(3, 1.0, 2.0, 3.0), vec2(3, 1.0, 2.0, 3.0), vec3(3, 2.0, 3.0, 4.0);
	Vector5 vec4(5, 1.0, 2.0, 3.0, 4.0, 5.0);

	CHECK(assert_equal(vec1, vec1, tol));
	CHECK(assert_equal(vec1, vec2, tol));
	CHECK(assert_equal(vec2, vec1, tol));
	CHECK(!vec1.equals(vec3, tol));
	CHECK(!vec3.equals(vec1, tol));
	CHECK(!vec1.equals(vec4, tol));
	CHECK(!vec4.equals(vec1, tol));
}

/* ************************************************************************* */
TEST( testFixedVector, static_constructors ) {
	Vector3 actZero = Vector3::zero();
	Vector3 expZero(3, 0.0, 0.0, 0.0);
	CHECK(assert_equal(expZero, actZero, tol));

	Vector3 actOnes = Vector3::ones();
	Vector3 expOnes(3, 1.0, 1.0, 1.0);
	CHECK(assert_equal(expOnes, actOnes, tol));

	Vector3 actRepeat = Vector3::repeat(2.3);
	Vector3 expRepeat(3, 2.3, 2.3, 2.3);
	CHECK(assert_equal(expRepeat, actRepeat, tol));

	Vector3 actBasis = Vector3::basis(1);
	Vector3 expBasis(3, 0.0, 1.0, 0.0);
	CHECK(assert_equal(expBasis, actBasis, tol));

	Vector3 actDelta = Vector3::delta(1, 2.3);
	Vector3 expDelta(3, 0.0, 2.3, 0.0);
	CHECK(assert_equal(expDelta, actDelta, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


