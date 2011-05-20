/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVectorValues.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Sep 16, 2010
 */

#include <vector>

#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Permutation.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

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
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
