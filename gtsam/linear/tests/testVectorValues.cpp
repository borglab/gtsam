/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVectorValues.cpp
 * @author  Richard Roberts
 * @date    Sep 16, 2010
 */

#include <boost/assign/std/vector.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Permutation.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

/* ************************************************************************* */
TEST(VectorValues, constructor) {
	vector<size_t> dims;
	dims.push_back(1);
	dims.push_back(2);
	dims.push_back(2);
	double v[] = {1., 2., 3., 4., 5.};

	VectorValues actual(dims, v);
	LONGS_EQUAL(3, actual.size());
	DOUBLES_EQUAL(1., actual[0][0], 1e-15);
	DOUBLES_EQUAL(2., actual[1][0], 1e-15);
	DOUBLES_EQUAL(4., actual[2][0], 1e-15);
}

/* ************************************************************************* */
TEST(VectorValues, standard) {
  Vector v1 = Vector_(3, 1.0,2.0,3.0);
  Vector v2 = Vector_(2, 4.0,5.0);
  Vector v3 = Vector_(4, 6.0,7.0,8.0,9.0);

  vector<size_t> dims(3); dims[0]=3; dims[1]=2; dims[2]=4;
  VectorValues combined(dims);
  EXPECT_LONGS_EQUAL(3, combined.size());
  EXPECT_LONGS_EQUAL(9, combined.dim());
  EXPECT_LONGS_EQUAL(9, combined.dimCapacity());
  EXPECT_LONGS_EQUAL(3, combined.dim(0));
  EXPECT_LONGS_EQUAL(2, combined.dim(1));
  EXPECT_LONGS_EQUAL(4, combined.dim(2));
  combined[0] = v1;
  combined[1] = v2;
  combined[2] = v3;

  CHECK(assert_equal(combined[0], v1))
  CHECK(assert_equal(combined[1], v2))
  CHECK(assert_equal(combined[2], v3))

  VectorValues incremental;
  incremental.reserve(3, 9);
  incremental.push_back_preallocated(v1);
  incremental.push_back_preallocated(v2);
  incremental.push_back_preallocated(v3);

  CHECK(assert_equal(incremental[0], v1))
  CHECK(assert_equal(incremental[1], v2))
  CHECK(assert_equal(incremental[2], v3))
}

/* ************************************************************************* */
TEST(VectorValues, permuted_combined) {
  Vector v1 = Vector_(3, 1.0,2.0,3.0);
  Vector v2 = Vector_(2, 4.0,5.0);
  Vector v3 = Vector_(4, 6.0,7.0,8.0,9.0);

  vector<size_t> dims(3); dims[0]=3; dims[1]=2; dims[2]=4;
  VectorValues combined(dims);
  combined[0] = v1;
  combined[1] = v2;
  combined[2] = v3;

  Permutation perm1(3);
  perm1[0] = 1;
  perm1[1] = 2;
  perm1[2] = 0;

  Permutation perm2(3);
  perm2[0] = 1;
  perm2[1] = 2;
  perm2[2] = 0;

  Permuted<VectorValues> permuted1(combined);
  CHECK(assert_equal(v1, permuted1[0]))
  CHECK(assert_equal(v2, permuted1[1]))
  CHECK(assert_equal(v3, permuted1[2]))

  permuted1.permute(perm1);
  CHECK(assert_equal(v1, permuted1[2]))
  CHECK(assert_equal(v2, permuted1[0]))
  CHECK(assert_equal(v3, permuted1[1]))

  permuted1.permute(perm2);
  CHECK(assert_equal(v1, permuted1[1]))
  CHECK(assert_equal(v2, permuted1[2]))
  CHECK(assert_equal(v3, permuted1[0]))

  Permuted<VectorValues> permuted2(perm1, combined);
  CHECK(assert_equal(v1, permuted2[2]))
  CHECK(assert_equal(v2, permuted2[0]))
  CHECK(assert_equal(v3, permuted2[1]))

  permuted2.permute(perm2);
  CHECK(assert_equal(v1, permuted2[1]))
  CHECK(assert_equal(v2, permuted2[2]))
  CHECK(assert_equal(v3, permuted2[0]))

}

/* ************************************************************************* */
TEST(VectorValues, permuted_incremental) {
  Vector v1 = Vector_(3, 1.0,2.0,3.0);
  Vector v2 = Vector_(2, 4.0,5.0);
  Vector v3 = Vector_(4, 6.0,7.0,8.0,9.0);

  VectorValues incremental;
  incremental.reserve(3, 9);
  incremental.push_back_preallocated(v1);
  incremental.push_back_preallocated(v2);
  incremental.push_back_preallocated(v3);

  Permutation perm1(3);
  perm1[0] = 1;
  perm1[1] = 2;
  perm1[2] = 0;

  Permutation perm2(3);
  perm2[0] = 1;
  perm2[1] = 2;
  perm2[2] = 0;

  Permuted<VectorValues> permuted1(incremental);
  CHECK(assert_equal(v1, permuted1[0]))
  CHECK(assert_equal(v2, permuted1[1]))
  CHECK(assert_equal(v3, permuted1[2]))

  permuted1.permute(perm1);
  CHECK(assert_equal(v1, permuted1[2]))
  CHECK(assert_equal(v2, permuted1[0]))
  CHECK(assert_equal(v3, permuted1[1]))

  permuted1.permute(perm2);
  CHECK(assert_equal(v1, permuted1[1]))
  CHECK(assert_equal(v2, permuted1[2]))
  CHECK(assert_equal(v3, permuted1[0]))

  Permuted<VectorValues> permuted2(perm1, incremental);
  CHECK(assert_equal(v1, permuted2[2]))
  CHECK(assert_equal(v2, permuted2[0]))
  CHECK(assert_equal(v3, permuted2[1]))

  permuted2.permute(perm2);
  CHECK(assert_equal(v1, permuted2[1]))
  CHECK(assert_equal(v2, permuted2[2]))
  CHECK(assert_equal(v3, permuted2[0]))

}

/* ************************************************************************* */
TEST(VectorValues, makeZero ) {
	VectorValues values(7, 2);
	EXPECT_LONGS_EQUAL(14, values.dim());
	EXPECT_LONGS_EQUAL(7, values.size());
	EXPECT_LONGS_EQUAL(14, values.vector().size());
	values.makeZero();
	EXPECT(assert_equal(zero(14), values.vector()));
}

/* ************************************************************************* */
TEST(VectorValues, reserve ) {
  Vector v1 = Vector_(3, 1.0,2.0,3.0);
  Vector v2 = Vector_(2, 4.0,5.0);
  Vector v3 = Vector_(4, 6.0,7.0,8.0,9.0);
  Vector v4(2); v4 << 10, 11;
  Vector v5(3); v5 << 12, 13, 14;

  // Expected has all 5 variables
  vector<size_t> dimsExp(5); dimsExp[0]=3; dimsExp[1]=2; dimsExp[2]=4; dimsExp[3]=2; dimsExp[4]=3;
  VectorValues expected(dimsExp);
  expected[0] = v1;
  expected[1] = v2;
  expected[2] = v3;
  expected[3] = v4;
  expected[4] = v5;

  // Start with 3 variables
  vector<size_t> dims(3); dims[0]=3; dims[1]=2; dims[2]=4;
  VectorValues actual(dims);
  actual[0] = v1;
  actual[1] = v2;
  actual[2] = v3;

  // Now expand to all 5
  actual.reserve(5, 14);
  actual.push_back_preallocated(v4);
  actual.push_back_preallocated(v5);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(VectorValues, range ) {
	VectorValues v(7,2);
	v.makeZero();
	v[1] = Vector_(2, 1.0, 2.0);
	v[2] = Vector_(2, 3.0, 4.0);
	v[3] = Vector_(2, 5.0, 6.0);

	vector<size_t> idx1, idx2;
	idx1 += 0, 1, 2, 3, 4, 5, 6; // ordered
	idx2 += 1, 0, 2;  // unordered

	// test access

	Vector actRange1 = v.range(idx1.begin(), idx1.begin() + 2);
	EXPECT(assert_equal(Vector_(4, 0.0, 0.0, 1.0, 2.0), actRange1));

	Vector actRange2 = v.range(idx1.begin()+1, idx1.begin()+2);
	EXPECT(assert_equal(Vector_(2, 1.0, 2.0), actRange2));

	Vector actRange3 = v.range(idx2.begin(), idx2.end());
	EXPECT(assert_equal(Vector_(6, 1.0, 2.0, 0.0, 0.0, 3.0, 4.0), actRange3));

	// test setting values
	VectorValues act1 = v, act2 = v, act3 = v;

	Vector a = Vector_(2, 0.1, 0.2);
	VectorValues exp1 = act1;	exp1[0] = a;
	act1.range(idx1.begin(), idx1.begin()+1, a);
	EXPECT(assert_equal(exp1, act1));

	Vector bc = Vector_(4, 0.1, 0.2, 0.3, 0.4);
	VectorValues exp2 = act2;
	exp2[2] = Vector_(2, 0.1, 0.2);
	exp2[3] = Vector_(2, 0.3, 0.4);
	act2.range(idx1.begin()+2, idx1.begin()+4, bc);
	EXPECT(assert_equal(exp2, act2));

	Vector def = Vector_(6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
	VectorValues exp3 = act3;
	exp3[1] = Vector_(2, 0.1, 0.2);
	exp3[0] = Vector_(2, 0.3, 0.4);
	exp3[2] = Vector_(2, 0.5, 0.6);
	act3.range(idx2.begin(), idx2.end(), def);
	EXPECT(assert_equal(exp3, act3));
}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
