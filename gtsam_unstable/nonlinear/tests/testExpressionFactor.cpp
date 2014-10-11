/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpressionFactor.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam_unstable/slam/expressions.h>
#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

Point2 measured(-17, 30);
SharedNoiseModel model = noiseModel::Unit::Create(2);

///* ************************************************************************* */
//// Leaf
//TEST(ExpressionFactor, leaf) {
//
//  // Create some values
//  Values values;
//  values.insert(2, Point2(3, 5));
//
//  JacobianFactor expected( //
//      2, (Matrix(2, 2) << 1, 0, 0, 1), //
//      (Vector(2) << -3, -5));
//
//  // Create leaves
//  Point2_ p(2);
//
//  // Concise version
//  ExpressionFactor<Point2> f(model, Point2(0, 0), p);
//  EXPECT_LONGS_EQUAL(2, f.dim());
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf, 1e-9));
//}
//
///* ************************************************************************* */
//// non-zero noise model
//TEST(ExpressionFactor, model) {
//
//  // Create some values
//  Values values;
//  values.insert(2, Point2(3, 5));
//
//  JacobianFactor expected( //
//      2, (Matrix(2, 2) << 10, 0, 0, 100), //
//      (Vector(2) << -30, -500));
//
//  // Create leaves
//  Point2_ p(2);
//
//  // Concise version
//  SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.01));
//
//  ExpressionFactor<Point2> f(model, Point2(0, 0), p);
//  EXPECT_LONGS_EQUAL(2, f.dim());
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf, 1e-9));
//}
//
///* ************************************************************************* */
//// Unary(Leaf))
//TEST(ExpressionFactor, unary) {
//
//  // Create some values
//  Values values;
//  values.insert(2, Point3(0, 0, 1));
//
//  JacobianFactor expected( //
//      2, (Matrix(2, 3) << 1, 0, 0, 0, 1, 0), //
//      (Vector(2) << -17, 30));
//
//  // Create leaves
//  Point3_ p(2);
//
//  // Concise version
//  ExpressionFactor<Point2> f(model, measured, project(p));
//  EXPECT_LONGS_EQUAL(2, f.dim());
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf, 1e-9));
//}
/* ************************************************************************* */
struct TestBinaryExpression {
  static Point2 myUncal(const Cal3_S2& K, const Point2& p,
      boost::optional<Matrix25&> Dcal, boost::optional<Matrix2&> Dp) {
    return K.uncalibrate(p, Dcal, Dp);
  }
  Cal3_S2_ K_;
  Point2_ p_;
  BinaryExpression<Point2, Cal3_S2, Point2> binary_;
  TestBinaryExpression() :
      K_(1), p_(2), binary_(myUncal, K_, p_) {
  }
};
/* ************************************************************************* */
// Binary(Leaf,Leaf)
TEST(ExpressionFactor, binary) {

  typedef BinaryExpression<Point2, Cal3_S2, Point2> Binary;
  TestBinaryExpression tester;

  // Create some values
  Values values;
  values.insert(1, Cal3_S2());
  values.insert(2, Point2(0, 0));

  // Expected Jacobians
  Matrix25 expected25;
  expected25 << 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  Matrix2 expected22;
  expected22 << 1, 0, 0, 1;

  // traceRaw will fill raw with [Trace<Point2> | Binary::Record]
  EXPECT_LONGS_EQUAL(8, sizeof(double));
  EXPECT_LONGS_EQUAL(16, sizeof(ExecutionTrace<Point2>));
  EXPECT_LONGS_EQUAL(16, sizeof(ExecutionTrace<Cal3_S2>));
  EXPECT_LONGS_EQUAL(2*5*8, sizeof(Binary::JacobianTA1));
  EXPECT_LONGS_EQUAL(2*2*8, sizeof(Binary::JacobianTA2));
  size_t expectedRecordSize = 16 + 2 * 16 + 80 + 32;
  EXPECT_LONGS_EQUAL(expectedRecordSize, sizeof(Binary::Record));
  size_t size = sizeof(Binary::Record);
  // Use Variable Length Array, allocated on stack by gcc
  // Note unclear for Clang: http://clang.llvm.org/compatibility.html#vla
  char raw[size];
  ExecutionTrace<Point2> trace;
  Point2 value = tester.binary_.traceExecution(values, trace, raw);
  // trace.print();

  // Check matrices
  boost::optional<Binary::Record*> p = trace.record<Binary::Record>();
  CHECK(p);
  EXPECT( assert_equal(expected25, (Matrix)(*p)->dTdA1, 1e-9));
  EXPECT( assert_equal(expected22, (Matrix)(*p)->dTdA2, 1e-9));
}
///* ************************************************************************* */
//// Unary(Binary(Leaf,Leaf))
//TEST(ExpressionFactor, shallow) {
//
//  // Create some values
//  Values values;
//  values.insert(1, Pose3());
//  values.insert(2, Point3(0, 0, 1));
//
//  // Create old-style factor to create expected value and derivatives
//  GenericProjectionFactor<Pose3, Point3> old(measured, model, 1, 2,
//      boost::make_shared<Cal3_S2>());
//  double expected_error = old.error(values);
//  GaussianFactor::shared_ptr expected = old.linearize(values);
//
//  // Create leaves
//  Pose3_ x(1);
//  Point3_ p(2);
//
//  // Concise version
//  ExpressionFactor<Point2> f2(model, measured, project(transform_to(x, p)));
//  EXPECT_DOUBLES_EQUAL(expected_error, f2.error(values), 1e-9);
//  EXPECT_LONGS_EQUAL(2, f2.dim());
//  boost::shared_ptr<GaussianFactor> gf2 = f2.linearize(values);
//  EXPECT( assert_equal(*expected, *gf2, 1e-9));
//}
//
///* ************************************************************************* */
//// Binary(Leaf,Unary(Binary(Leaf,Leaf)))
//TEST(ExpressionFactor, tree) {
//
//  // Create some values
//  Values values;
//  values.insert(1, Pose3());
//  values.insert(2, Point3(0, 0, 1));
//  values.insert(3, Cal3_S2());
//
//  // Create old-style factor to create expected value and derivatives
//  GeneralSFMFactor2<Cal3_S2> old(measured, model, 1, 2, 3);
//  double expected_error = old.error(values);
//  GaussianFactor::shared_ptr expected = old.linearize(values);
//
//  // Create leaves
//  Pose3_ x(1);
//  Point3_ p(2);
//  Cal3_S2_ K(3);
//
//  // Create expression tree
//  Point3_ p_cam(x, &Pose3::transform_to, p);
//  Point2_ xy_hat(PinholeCamera<Cal3_S2>::project_to_camera, p_cam);
//  Point2_ uv_hat(K, &Cal3_S2::uncalibrate, xy_hat);
//
//  // Create factor and check value, dimension, linearization
//  ExpressionFactor<Point2> f(model, measured, uv_hat);
//  EXPECT_DOUBLES_EQUAL(expected_error, f.error(values), 1e-9);
//  EXPECT_LONGS_EQUAL(2, f.dim());
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  EXPECT( assert_equal(*expected, *gf, 1e-9));
//
//  // Concise version
//  ExpressionFactor<Point2> f2(model, measured,
//      uncalibrate(K, project(transform_to(x, p))));
//  EXPECT_DOUBLES_EQUAL(expected_error, f2.error(values), 1e-9);
//  EXPECT_LONGS_EQUAL(2, f2.dim());
//  boost::shared_ptr<GaussianFactor> gf2 = f2.linearize(values);
//  EXPECT( assert_equal(*expected, *gf2, 1e-9));
//
//  TernaryExpression<Point2, Pose3, Point3, Cal3_S2>::Function fff = project6;
//
//  // Try ternary version
//  ExpressionFactor<Point2> f3(model, measured, project3(x, p, K));
//  EXPECT_DOUBLES_EQUAL(expected_error, f3.error(values), 1e-9);
//  EXPECT_LONGS_EQUAL(2, f3.dim());
//  boost::shared_ptr<GaussianFactor> gf3 = f3.linearize(values);
//  EXPECT( assert_equal(*expected, *gf3, 1e-9));
//}
//
///* ************************************************************************* */
//
//TEST(ExpressionFactor, compose1) {
//
//  // Create expression
//  Rot3_ R1(1), R2(2);
//  Rot3_ R3 = R1 * R2;
//
//  // Create factor
//  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), R3);
//
//  // Create some values
//  Values values;
//  values.insert(1, Rot3());
//  values.insert(2, Rot3());
//
//  // Check unwhitenedError
//  std::vector<Matrix> H(2);
//  Vector actual = f.unwhitenedError(values, H);
//  EXPECT( assert_equal(eye(3), H[0],1e-9));
//  EXPECT( assert_equal(eye(3), H[1],1e-9));
//
//  // Check linearization
//  JacobianFactor expected(1, eye(3), 2, eye(3), zero(3));
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf,1e-9));
//}
//
///* ************************************************************************* */
//// Test compose with arguments referring to the same rotation
//TEST(ExpressionFactor, compose2) {
//
//  // Create expression
//  Rot3_ R1(1), R2(1);
//  Rot3_ R3 = R1 * R2;
//
//  // Create factor
//  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), R3);
//
//  // Create some values
//  Values values;
//  values.insert(1, Rot3());
//
//  // Check unwhitenedError
//  std::vector<Matrix> H(1);
//  Vector actual = f.unwhitenedError(values, H);
//  EXPECT_LONGS_EQUAL(1, H.size());
//  EXPECT( assert_equal(2*eye(3), H[0],1e-9));
//
//  // Check linearization
//  JacobianFactor expected(1, 2 * eye(3), zero(3));
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf,1e-9));
//}
//
///* ************************************************************************* */
//// Test compose with one arguments referring to a constant same rotation
//TEST(ExpressionFactor, compose3) {
//
//  // Create expression
//  Rot3_ R1(Rot3::identity()), R2(3);
//  Rot3_ R3 = R1 * R2;
//
//  // Create factor
//  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), R3);
//
//  // Create some values
//  Values values;
//  values.insert(3, Rot3());
//
//  // Check unwhitenedError
//  std::vector<Matrix> H(1);
//  Vector actual = f.unwhitenedError(values, H);
//  EXPECT_LONGS_EQUAL(1, H.size());
//  EXPECT( assert_equal(eye(3), H[0],1e-9));
//
//  // Check linearization
//  JacobianFactor expected(3, eye(3), zero(3));
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf,1e-9));
//}
//
///* ************************************************************************* */
//// Test compose with three arguments
//Rot3 composeThree(const Rot3& R1, const Rot3& R2, const Rot3& R3,
//    boost::optional<Matrix3&> H1, boost::optional<Matrix3&> H2,
//    boost::optional<Matrix3&> H3) {
//  // return dummy derivatives (not correct, but that's ok for testing here)
//  if (H1)
//    *H1 = eye(3);
//  if (H2)
//    *H2 = eye(3);
//  if (H3)
//    *H3 = eye(3);
//  return R1 * (R2 * R3);
//}
//
//TEST(ExpressionFactor, composeTernary) {
//
//  // Create expression
//  Rot3_ A(1), B(2), C(3);
//  Rot3_ ABC(composeThree, A, B, C);
//
//  // Create factor
//  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), ABC);
//
//  // Create some values
//  Values values;
//  values.insert(1, Rot3());
//  values.insert(2, Rot3());
//  values.insert(3, Rot3());
//
//  // Check unwhitenedError
//  std::vector<Matrix> H(3);
//  Vector actual = f.unwhitenedError(values, H);
//  EXPECT_LONGS_EQUAL(3, H.size());
//  EXPECT( assert_equal(eye(3), H[0],1e-9));
//  EXPECT( assert_equal(eye(3), H[1],1e-9));
//  EXPECT( assert_equal(eye(3), H[2],1e-9));
//
//  // Check linearization
//  JacobianFactor expected(1, eye(3), 2, eye(3), 3, eye(3), zero(3));
//  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
//  boost::shared_ptr<JacobianFactor> jf = //
//      boost::dynamic_pointer_cast<JacobianFactor>(gf);
//  EXPECT( assert_equal(expected, *jf,1e-9));
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

