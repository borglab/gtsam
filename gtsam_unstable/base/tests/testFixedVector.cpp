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

#include <gtsam_unstable/base/FixedVector.h>

using namespace gtsam;

typedef FixedVector<5> TestVector5;
typedef FixedVector<3> TestVector3;

static const double tol = 1e-9;

/* ************************************************************************* */
TEST( testFixedVector, conversions ) {
  double data1[] = {1.0, 2.0, 3.0};
  Vector v1  = (Vector(3) << data1[0], data1[1], data1[2]).finished();
  TestVector3 fv1(v1), fv2(data1);

  Vector actFv2(fv2);
  EXPECT(assert_equal(v1, actFv2));
}

/* ************************************************************************* */
TEST( testFixedVector, variable_constructor ) {
  TestVector3 act(Vector3(1.0, 2.0, 3.0));
  EXPECT_DOUBLES_EQUAL(1.0, act(0), tol);
  EXPECT_DOUBLES_EQUAL(2.0, act(1), tol);
  EXPECT_DOUBLES_EQUAL(3.0, act(2), tol);
}

/* ************************************************************************* */
TEST( testFixedVector, equals ) {
  TestVector3 vec1(Vector3(1.0, 2.0, 3.0)), vec2(Vector3(1.0, 2.0, 3.0)),
      vec3(Vector3(2.0, 3.0, 4.0));
  TestVector5 vec4((Vector(5) << 1.0, 2.0, 3.0, 4.0, 5.0).finished());

  EXPECT(assert_equal(vec1, vec1, tol));
  EXPECT(assert_equal(vec1, vec2, tol));
  EXPECT(assert_equal(vec2, vec1, tol));
  EXPECT(!vec1.equals(vec3, tol));
  EXPECT(!vec3.equals(vec1, tol));
  EXPECT(!vec1.equals(vec4, tol));
  EXPECT(!vec4.equals(vec1, tol));
}

/* ************************************************************************* */
TEST( testFixedVector, static_constructors ) {
  TestVector3 actZero = TestVector3::zero();
  TestVector3 expZero(Vector3(0.0, 0.0, 0.0));
  EXPECT(assert_equal(expZero, actZero, tol));

  TestVector3 actOnes = TestVector3::ones();
  TestVector3 expOnes(Vector3(1.0, 1.0, 1.0));
  EXPECT(assert_equal(expOnes, actOnes, tol));

  TestVector3 actRepeat = TestVector3::repeat(2.3);
  TestVector3 expRepeat(Vector3(2.3, 2.3, 2.3));
  EXPECT(assert_equal(expRepeat, actRepeat, tol));

  TestVector3 actBasis = TestVector3::basis(1);
  TestVector3 expBasis(Vector3(0.0, 1.0, 0.0));
  EXPECT(assert_equal(expBasis, actBasis, tol));

  TestVector3 actDelta = TestVector3::delta(1, 2.3);
  TestVector3 expDelta(Vector3(0.0, 2.3, 0.0));
  EXPECT(assert_equal(expDelta, actDelta, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


