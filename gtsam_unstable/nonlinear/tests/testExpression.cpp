/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testExpression.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam_unstable/nonlinear/Expression.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/LieScalar.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

using namespace std;
using namespace gtsam;

typedef pair<Key,size_t> Pair;

/* ************************************************************************* */

template<class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p,
    boost::optional<Matrix25&> Dcal, boost::optional<Matrix2&> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
}

static const Rot3 someR = Rot3::RzRyRx(1, 2, 3);

/* ************************************************************************* */
// Constant
TEST(Expression, constant) {
  Expression<Rot3> R(someR);
  Values values;
  JacobianMap actualMap;
  Rot3 actual = R.value(values, actualMap);
  EXPECT(assert_equal(someR, actual));
  JacobianMap expected;
  EXPECT(actualMap == expected);
}

/* ************************************************************************* */
// Leaf
TEST(Expression, leaf) {
  Expression<Rot3> R(100);
  Values values;
  values.insert(100, someR);

  JacobianMap expected;
  Matrix H = eye(3);
  expected.insert(make_pair(100,H.block(0,0,3,3)));

  JacobianMap actualMap2;
  actualMap2.insert(make_pair(100,H.block(0,0,3,3)));
  Rot3 actual2 = R.reverse(values, actualMap2);
  EXPECT(assert_equal(someR, actual2));
  EXPECT(actualMap2 == expected);
}

/* ************************************************************************* */

//TEST(Expression, nullaryMethod) {
//  Expression<Point3> p(67);
//  Expression<LieScalar> norm(p, &Point3::norm);
//  Values values;
//  values.insert(67,Point3(3,4,5));
//  Augmented<LieScalar> a = norm.augmented(values);
//  EXPECT(a.value() == sqrt(50));
//  JacobianMap expected;
//  expected[67] = (Matrix(1,3) << 3/sqrt(50),4/sqrt(50),5/sqrt(50));
//  EXPECT(assert_equal(expected.at(67),a.jacobians().at(67)));
//}
/* ************************************************************************* */
// Binary(Leaf,Leaf)
namespace binary {
// Create leaves
Expression<Pose3> x(1);
Expression<Point3> p(2);
Expression<Point3> p_cam(x, &Pose3::transform_to, p);
}
/* ************************************************************************* */
// keys
TEST(Expression, BinaryKeys) {
  set<Key> expected = list_of(1)(2);
  EXPECT(expected == binary::p_cam.keys());
}
/* ************************************************************************* */
// dimensions
TEST(Expression, BinaryDimensions) {
  map<Key, size_t> expected = map_list_of(1, 6)(2, 3), //
  actual = binary::p_cam.dimensions();
  EXPECT_LONGS_EQUAL(expected.size(),actual.size());
  BOOST_FOREACH(Pair pair, actual)
    EXPECT_LONGS_EQUAL(expected[pair.first],pair.second);
}
/* ************************************************************************* */
// Binary(Leaf,Unary(Binary(Leaf,Leaf)))
namespace tree {
using namespace binary;
// Create leaves
Expression<Cal3_S2> K(3);

// Create expression tree
Expression<Point2> projection(PinholeCamera<Cal3_S2>::project_to_camera, p_cam);
Expression<Point2> uv_hat(uncalibrate<Cal3_S2>, K, projection);
}
/* ************************************************************************* */
// keys
TEST(Expression, TreeKeys) {
  set<Key> expected = list_of(1)(2)(3);
  EXPECT(expected == tree::uv_hat.keys());
}
/* ************************************************************************* */
// dimensions
TEST(Expression, TreeDimensions) {
  map<Key, size_t> expected = map_list_of(1, 6)(2, 3)(3, 5), //
  actual = tree::uv_hat.dimensions();
  EXPECT_LONGS_EQUAL(expected.size(),actual.size());
  BOOST_FOREACH(Pair pair, actual)
    EXPECT_LONGS_EQUAL(expected[pair.first],pair.second);
}
/* ************************************************************************* */

TEST(Expression, compose1) {

  // Create expression
  Expression<Rot3> R1(1), R2(2);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  set<Key> expected = list_of(1)(2);
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with arguments referring to the same rotation
TEST(Expression, compose2) {

  // Create expression
  Expression<Rot3> R1(1), R2(1);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  set<Key> expected = list_of(1);
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with one arguments referring to constant rotation
TEST(Expression, compose3) {

  // Create expression
  Expression<Rot3> R1(Rot3::identity()), R2(3);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  set<Key> expected = list_of(3);
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test with ternary function
Rot3 composeThree(const Rot3& R1, const Rot3& R2, const Rot3& R3,
    boost::optional<Matrix3&> H1, boost::optional<Matrix3&> H2,
    boost::optional<Matrix3&> H3) {
  // return dummy derivatives (not correct, but that's ok for testing here)
  if (H1)
    *H1 = eye(3);
  if (H2)
    *H2 = eye(3);
  if (H3)
    *H3 = eye(3);
  return R1 * (R2 * R3);
}

TEST(Expression, ternary) {

  // Create expression
  Expression<Rot3> A(1), B(2), C(3);
  Expression<Rot3> ABC(composeThree, A, B, C);

  // Check keys
  set<Key> expected = list_of(1)(2)(3);
  EXPECT(expected == ABC.keys());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

