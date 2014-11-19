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
  Rot3 actual = R.value(values);
  EXPECT(assert_equal(someR, actual));
  EXPECT_LONGS_EQUAL(0, R.traceSize())
}

/* ************************************************************************* */
// Leaf
TEST(Expression, Leaf) {
  Expression<Rot3> R(100);
  Values values;
  values.insert(100, someR);

  Rot3 actual2 = R.value(values);
  EXPECT(assert_equal(someR, actual2));
}

/* ************************************************************************* */
// Many Leaves
TEST(Expression, Leaves) {
  Values values;
  Point3 somePoint(1, 2, 3);
  values.insert(Symbol('p', 10), somePoint);
  std::vector<Expression<Point3> > points = createUnknowns<Point3>(10, 'p', 1);
  EXPECT(assert_equal(somePoint,points.back().value(values)));
}

/* ************************************************************************* */

//TEST(Expression, NullaryMethod) {
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
  map<Key, size_t> actual, expected = map_list_of<Key, size_t>(1, 6)(2, 3);
  binary::p_cam.dims(actual);
  EXPECT(actual==expected);
}
/* ************************************************************************* */
// dimensions
TEST(Expression, BinaryTraceSize) {
  typedef BinaryExpression<Point3, Pose3, Point3> Binary;
  size_t expectedTraceSize = sizeof(Binary::Record);
  EXPECT_LONGS_EQUAL(expectedTraceSize, binary::p_cam.traceSize());
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
  map<Key, size_t> actual, expected = map_list_of<Key, size_t>(1, 6)(2, 3)(3,
      5);
  tree::uv_hat.dims(actual);
  EXPECT(actual==expected);
}
/* ************************************************************************* */
// TraceSize
TEST(Expression, TreeTraceSize) {
  typedef UnaryExpression<Point2, Point3> Unary;
  typedef BinaryExpression<Point3, Pose3, Point3> Binary1;
  typedef BinaryExpression<Point2, Point2, Cal3_S2> Binary2;
  size_t expectedTraceSize = sizeof(Unary::Record) + sizeof(Binary1::Record)
      + sizeof(Binary2::Record);
  EXPECT_LONGS_EQUAL(expectedTraceSize, tree::uv_hat.traceSize());
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

