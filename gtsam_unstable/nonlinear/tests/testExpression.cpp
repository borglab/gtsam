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

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

template<class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p, boost::optional<Matrix&> Dcal,
    boost::optional<Matrix&> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
}

static const Rot3 someR = Rot3::RzRyRx(1,2,3);

/* ************************************************************************* */

TEST(Expression, constant) {
  Expression<Rot3> R(someR);
  Values values;
  Augmented<Rot3> a = R.augmented(values);
  EXPECT(assert_equal(someR, a.value()));
  JacobianMap expected;
  EXPECT(a.jacobians() == expected);
}

/* ************************************************************************* */

TEST(Expression, leaf) {
  Expression<Rot3> R(100);
  Values values;
  values.insert(100,someR);
  Augmented<Rot3> a = R.augmented(values);
  EXPECT(assert_equal(someR, a.value()));
  JacobianMap expected;
  expected[100] = eye(3);
  EXPECT(a.jacobians() == expected);
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

TEST(Expression, test) {

  // Test Constant expression
  Expression<int> c(0);

  // Create leaves
  Expression<Pose3> x(1);
  Expression<Point3> p(2);
  Expression<Cal3_S2> K(3);

  // Create expression tree
  Expression<Point3> p_cam(x, &Pose3::transform_to, p);
  Expression<Point2> projection(PinholeCamera<Cal3_S2>::project_to_camera,
      p_cam);
  Expression<Point2> uv_hat(uncalibrate<Cal3_S2>, K, projection);

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(1);
  expectedKeys.insert(2);
  expectedKeys.insert(3);
  EXPECT(expectedKeys == uv_hat.keys());
}

/* ************************************************************************* */

TEST(Expression, compose1) {

  // Create expression
  Expression<Rot3> R1(1), R2(2);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(1);
  expectedKeys.insert(2);
  EXPECT(expectedKeys == R3.keys());
}

/* ************************************************************************* */
// Test compose with arguments referring to the same rotation
TEST(Expression, compose2) {

  // Create expression
  Expression<Rot3> R1(1), R2(1);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(1);
  EXPECT(expectedKeys == R3.keys());
}

/* ************************************************************************* */
// Test compose with one arguments referring to constant rotation
TEST(Expression, compose3) {

  // Create expression
  Expression<Rot3> R1(Rot3::identity()), R2(3);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(3);
  EXPECT(expectedKeys == R3.keys());
}

/* ************************************************************************* */
// Test with ternary function
Rot3 composeThree(const Rot3& R1, const Rot3& R2, const Rot3& R3,
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3) {
  // return dummy derivatives (not correct, but that's ok for testing here)
  if (H1)
    *H1 = eye(3);
  if (H2)
    *H2 = eye(3);
  if (H3)
    *H3 = eye(3);
  return R1 * (R2 * R3);
}

//TEST(Expression, ternary) {
//
//  // Create expression
//  Expression<Rot3> A(1), B(2), C(3);
//  Expression<Rot3> ABC(composeThree, A, B, C);
//
//  // Check keys
//  std::set<Key> expectedKeys;
//  expectedKeys.insert(1);
//  expectedKeys.insert(2);
//  expectedKeys.insert(3);
//  EXPECT(expectedKeys == ABC.keys());
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

