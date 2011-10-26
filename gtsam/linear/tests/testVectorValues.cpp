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
TEST(VectorValues, insert) {

  // insert, with out-of-order indices
  VectorValues actual;
  actual.insert(0, Vector_(1, 1.0));
  actual.insert(1, Vector_(2, 2.0, 3.0));
  actual.insert(5, Vector_(2, 6.0, 7.0));
  actual.insert(2, Vector_(2, 4.0, 5.0));

  // Check dimensions
  LONGS_EQUAL(6, actual.size());
  LONGS_EQUAL(7, actual.dim());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(2, actual.dim(5));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(!actual.exists(3));
  EXPECT(!actual.exists(4));
  EXPECT(actual.exists(5));
  EXPECT(!actual.exists(6));

  // Check values
  EXPECT(assert_equal(Vector_(1, 1.0), actual[0]));
  EXPECT(assert_equal(Vector_(2, 2.0, 3.0), actual[1]));
  EXPECT(assert_equal(Vector_(2, 4.0, 5.0), actual[2]));
  EXPECT(assert_equal(Vector_(2, 6.0, 7.0), actual[5]));
  EXPECT(assert_equal(Vector_(7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), actual.vector()));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
  CHECK_EXCEPTION(actual.dim(3), out_of_range);
}

/* ************************************************************************* */
TEST(VectorValues, dimsConstructor) {
	vector<size_t> dims;
	dims.push_back(1);
	dims.push_back(2);
	dims.push_back(2);

	VectorValues actual(dims);
	actual[0] = Vector_(1, 1.0);
	actual[1] = Vector_(2, 2.0, 3.0);
	actual[2] = Vector_(2, 4.0, 5.0);

	// Check dimensions
	LONGS_EQUAL(3, actual.size());
	LONGS_EQUAL(5, actual.dim());
	LONGS_EQUAL(1, actual.dim(0));
	LONGS_EQUAL(2, actual.dim(1));
	LONGS_EQUAL(2, actual.dim(2));

	// Check values
	EXPECT(assert_equal(Vector_(1, 1.0), actual[0]));
  EXPECT(assert_equal(Vector_(2, 2.0, 3.0), actual[1]));
  EXPECT(assert_equal(Vector_(2, 4.0, 5.0), actual[2]));
  EXPECT(assert_equal(Vector_(5, 1.0, 2.0, 3.0, 4.0, 5.0), actual.vector()));
}

/* ************************************************************************* */
TEST(VectorValues, copyConstructor) {

  // insert, with out-of-order indices
  VectorValues original;
  original.insert(0, Vector_(1, 1.0));
  original.insert(1, Vector_(2, 2.0, 3.0));
  original.insert(5, Vector_(2, 6.0, 7.0));
  original.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues actual(original);

  // Check dimensions
  LONGS_EQUAL(6, actual.size());
  LONGS_EQUAL(7, actual.dim());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(2, actual.dim(5));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(!actual.exists(3));
  EXPECT(!actual.exists(4));
  EXPECT(actual.exists(5));
  EXPECT(!actual.exists(6));

  // Check values
  EXPECT(assert_equal(Vector_(1, 1.0), actual[0]));
  EXPECT(assert_equal(Vector_(2, 2.0, 3.0), actual[1]));
  EXPECT(assert_equal(Vector_(2, 4.0, 5.0), actual[2]));
  EXPECT(assert_equal(Vector_(2, 6.0, 7.0), actual[5]));
  EXPECT(assert_equal(Vector_(7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), actual.vector()));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
}

/* ************************************************************************* */
TEST(VectorValues, assignment) {

  // insert, with out-of-order indices
  VectorValues original;
  original.insert(0, Vector_(1, 1.0));
  original.insert(1, Vector_(2, 2.0, 3.0));
  original.insert(5, Vector_(2, 6.0, 7.0));
  original.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues actual = original;

  // Check dimensions
  LONGS_EQUAL(6, actual.size());
  LONGS_EQUAL(7, actual.dim());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(2, actual.dim(5));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(!actual.exists(3));
  EXPECT(!actual.exists(4));
  EXPECT(actual.exists(5));
  EXPECT(!actual.exists(6));

  // Check values
  EXPECT(assert_equal(Vector_(1, 1.0), actual[0]));
  EXPECT(assert_equal(Vector_(2, 2.0, 3.0), actual[1]));
  EXPECT(assert_equal(Vector_(2, 4.0, 5.0), actual[2]));
  EXPECT(assert_equal(Vector_(2, 6.0, 7.0), actual[5]));
  EXPECT(assert_equal(Vector_(7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), actual.vector()));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
}

/* ************************************************************************* */
TEST(VectorValues, SameStructure) {
  // insert, with out-of-order indices
  VectorValues original;
  original.insert(0, Vector_(1, 1.0));
  original.insert(1, Vector_(2, 2.0, 3.0));
  original.insert(5, Vector_(2, 6.0, 7.0));
  original.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues actual(VectorValues::SameStructure(original));

  // Check dimensions
  LONGS_EQUAL(6, actual.size());
  LONGS_EQUAL(7, actual.dim());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(2, actual.dim(5));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(!actual.exists(3));
  EXPECT(!actual.exists(4));
  EXPECT(actual.exists(5));
  EXPECT(!actual.exists(6));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
}

/* ************************************************************************* */
TEST(VectorValues, resizeLike) {
  // insert, with out-of-order indices
  VectorValues original;
  original.insert(0, Vector_(1, 1.0));
  original.insert(1, Vector_(2, 2.0, 3.0));
  original.insert(5, Vector_(2, 6.0, 7.0));
  original.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues actual(10, 3);
  actual.resizeLike(original);

  // Check dimensions
  LONGS_EQUAL(6, actual.size());
  LONGS_EQUAL(7, actual.dim());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(2, actual.dim(5));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(!actual.exists(3));
  EXPECT(!actual.exists(4));
  EXPECT(actual.exists(5));
  EXPECT(!actual.exists(6));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
}

/* ************************************************************************* */
TEST(VectorValues, append) {
  // insert
  VectorValues actual;
  actual.insert(0, Vector_(1, 1.0));
  actual.insert(1, Vector_(2, 2.0, 3.0));
  actual.insert(2, Vector_(2, 4.0, 5.0));

  // append
  vector<size_t> dims(2);
  dims[0] = 3;
  dims[1] = 5;
  actual.append(dims);

  // Check dimensions
  LONGS_EQUAL(5, actual.size());
  LONGS_EQUAL(13, actual.dim());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(3, actual.dim(3));
  LONGS_EQUAL(5, actual.dim(4));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(actual.exists(3));
  EXPECT(actual.exists(4));
  EXPECT(!actual.exists(5));

  // Check values
  EXPECT(assert_equal(Vector_(1, 1.0), actual[0]));
  EXPECT(assert_equal(Vector_(2, 2.0, 3.0), actual[1]));
  EXPECT(assert_equal(Vector_(2, 4.0, 5.0), actual[2]));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(3, Vector()), invalid_argument);
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

///* ************************************************************************* */
//TEST(VectorValues, range ) {
//	VectorValues v(7,2);
//	v.makeZero();
//	v[1] = Vector_(2, 1.0, 2.0);
//	v[2] = Vector_(2, 3.0, 4.0);
//	v[3] = Vector_(2, 5.0, 6.0);
//
//	vector<size_t> idx1, idx2;
//	idx1 += 0, 1, 2, 3, 4, 5, 6; // ordered
//	idx2 += 1, 0, 2;  // unordered
//
//	// test access
//
//	Vector actRange1 = v.range(idx1.begin(), idx1.begin() + 2);
//	EXPECT(assert_equal(Vector_(4, 0.0, 0.0, 1.0, 2.0), actRange1));
//
//	Vector actRange2 = v.range(idx1.begin()+1, idx1.begin()+2);
//	EXPECT(assert_equal(Vector_(2, 1.0, 2.0), actRange2));
//
//	Vector actRange3 = v.range(idx2.begin(), idx2.end());
//	EXPECT(assert_equal(Vector_(6, 1.0, 2.0, 0.0, 0.0, 3.0, 4.0), actRange3));
//
//	// test setting values
//	VectorValues act1 = v, act2 = v, act3 = v;
//
//	Vector a = Vector_(2, 0.1, 0.2);
//	VectorValues exp1 = act1;	exp1[0] = a;
//	act1.range(idx1.begin(), idx1.begin()+1, a);
//	EXPECT(assert_equal(exp1, act1));
//
//	Vector bc = Vector_(4, 0.1, 0.2, 0.3, 0.4);
//	VectorValues exp2 = act2;
//	exp2[2] = Vector_(2, 0.1, 0.2);
//	exp2[3] = Vector_(2, 0.3, 0.4);
//	act2.range(idx1.begin()+2, idx1.begin()+4, bc);
//	EXPECT(assert_equal(exp2, act2));
//
//	Vector def = Vector_(6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
//	VectorValues exp3 = act3;
//	exp3[1] = Vector_(2, 0.1, 0.2);
//	exp3[0] = Vector_(2, 0.3, 0.4);
//	exp3[2] = Vector_(2, 0.5, 0.6);
//	act3.range(idx2.begin(), idx2.end(), def);
//	EXPECT(assert_equal(exp3, act3));
//}

/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
