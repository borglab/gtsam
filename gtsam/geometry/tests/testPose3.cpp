/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPose3.cpp
 * @brief  Unit tests for Pose3 class
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/testLie.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/TestableAssertions.h>

#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;
using namespace std::placeholders;

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Pose3)
GTSAM_CONCEPT_LIE_INST(Pose3)

static const Point3 P(0.2,0.7,-2);
static const Rot3 R = Rot3::Rodrigues(0.3,0,0);
static const Point3 P2(3.5,-8.2,4.2);
static const Pose3 T(R,P2);
static const Pose3 T2(Rot3::Rodrigues(0.3,0.2,0.1),P2);
static const Pose3 T3(Rot3::Rodrigues(-90, 0, 0), Point3(1, 2, 3));
static const double tol=1e-5;

/* ************************************************************************* */
TEST( Pose3, equals)
{
  Pose3 pose2 = T3;
  EXPECT(T3.equals(pose2));
  Pose3 origin;
  EXPECT(!T3.equals(origin));
}

/* ************************************************************************* */
TEST( Pose3, constructors)
{
  Pose3 expected(Rot3::Rodrigues(0,0,3),Point3(1,2,0));
  Pose2 pose2(1,2,3);
  EXPECT(assert_equal(expected,Pose3(pose2)));
}

/* ************************************************************************* */
#ifndef GTSAM_POSE3_EXPMAP
TEST( Pose3, retract_first_order)
{
  Pose3 id;
  Vector v = Z_6x1;
  v(0) = 0.3;
  EXPECT(assert_equal(Pose3(R, Point3(0,0,0)), id.retract(v),1e-2));
  v(3)=0.2;v(4)=0.7;v(5)=-2;
  EXPECT(assert_equal(Pose3(R, P),id.retract(v),1e-2));
}
#endif
/* ************************************************************************* */
TEST( Pose3, retract_expmap)
{
  Vector v = Z_6x1; v(0) = 0.3;
  Pose3 pose = Pose3::Expmap(v);
  EXPECT(assert_equal(Pose3(R, Point3(0,0,0)), pose, 1e-2));
  EXPECT(assert_equal(v,Pose3::Logmap(pose),1e-2));
}

/* ************************************************************************* */
TEST( Pose3, expmap_a_full)
{
  Pose3 id;
  Vector v = Z_6x1;
  v(0) = 0.3;
  EXPECT(assert_equal(expmap_default<Pose3>(id, v), Pose3(R, Point3(0,0,0))));
  v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
  EXPECT(assert_equal(Pose3(R, P),expmap_default<Pose3>(id, v),1e-5));
}

/* ************************************************************************* */
TEST( Pose3, expmap_a_full2)
{
  Pose3 id;
  Vector v = Z_6x1;
  v(0) = 0.3;
  EXPECT(assert_equal(expmap_default<Pose3>(id, v), Pose3(R, Point3(0,0,0))));
  v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
  EXPECT(assert_equal(Pose3(R, P),expmap_default<Pose3>(id, v),1e-5));
}

/* ************************************************************************* */
TEST(Pose3, expmap_b)
{
  Pose3 p1(Rot3(), Point3(100, 0, 0));
  Pose3 p2 = p1.retract((Vector(6) << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0).finished());
  Pose3 expected(Rot3::Rodrigues(0.0, 0.0, 0.1), Point3(100.0, 0.0, 0.0));
  EXPECT(assert_equal(expected, p2,1e-2));
}

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screwPose3 {
  double a=0.3, c=cos(a), s=sin(a), w=0.3;
  Vector xi = (Vector(6) << 0.0, 0.0, w, w, 0.0, 1.0).finished();
  Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
  Point3 expectedT(0.29552, 0.0446635, 1);
  Pose3 expected(expectedR, expectedT);
}

/* ************************************************************************* */
// Checks correct exponential map (Expmap) with brute force matrix exponential
TEST(Pose3, expmap_c_full)
{
  EXPECT(assert_equal(screwPose3::expected, expm<Pose3>(screwPose3::xi),1e-6));
  EXPECT(assert_equal(screwPose3::expected, Pose3::Expmap(screwPose3::xi),1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(Pose3, Adjoint_full)
{
  Pose3 expected = T * Pose3::Expmap(screwPose3::xi) * T.inverse();
  Vector xiprime = T.Adjoint(screwPose3::xi);
  EXPECT(assert_equal(expected, Pose3::Expmap(xiprime), 1e-6));

  Pose3 expected2 = T2 * Pose3::Expmap(screwPose3::xi) * T2.inverse();
  Vector xiprime2 = T2.Adjoint(screwPose3::xi);
  EXPECT(assert_equal(expected2, Pose3::Expmap(xiprime2), 1e-6));

  Pose3 expected3 = T3 * Pose3::Expmap(screwPose3::xi) * T3.inverse();
  Vector xiprime3 = T3.Adjoint(screwPose3::xi);
  EXPECT(assert_equal(expected3, Pose3::Expmap(xiprime3), 1e-6));
}

/* ************************************************************************* */
// Check Adjoint numerical derivatives
TEST(Pose3, Adjoint_jacobians)
{
  Vector6 xi = (Vector6() << 0.1, 1.2, 2.3, 3.1, 1.4, 4.5).finished();

  // Check evaluation sanity check
  EQUALITY(static_cast<gtsam::Vector>(T.AdjointMap() * xi), T.Adjoint(xi));
  EQUALITY(static_cast<gtsam::Vector>(T2.AdjointMap() * xi), T2.Adjoint(xi));
  EQUALITY(static_cast<gtsam::Vector>(T3.AdjointMap() * xi), T3.Adjoint(xi));

  // Check jacobians
  Matrix6 actualH1, actualH2, expectedH1, expectedH2;
  std::function<Vector6(const Pose3&, const Vector6&)> Adjoint_proxy =
      [&](const Pose3& T, const Vector6& xi) { return T.Adjoint(xi); };

  T.Adjoint(xi, actualH1, actualH2);
  expectedH1 = numericalDerivative21(Adjoint_proxy, T, xi);
  expectedH2 = numericalDerivative22(Adjoint_proxy, T, xi);
  EXPECT(assert_equal(expectedH1, actualH1));
  EXPECT(assert_equal(expectedH2, actualH2));

  T2.Adjoint(xi, actualH1, actualH2);
  expectedH1 = numericalDerivative21(Adjoint_proxy, T2, xi);
  expectedH2 = numericalDerivative22(Adjoint_proxy, T2, xi);
  EXPECT(assert_equal(expectedH1, actualH1));
  EXPECT(assert_equal(expectedH2, actualH2));

  T3.Adjoint(xi, actualH1, actualH2);
  expectedH1 = numericalDerivative21(Adjoint_proxy, T3, xi);
  expectedH2 = numericalDerivative22(Adjoint_proxy, T3, xi);
  EXPECT(assert_equal(expectedH1, actualH1));
  EXPECT(assert_equal(expectedH2, actualH2));
}

/* ************************************************************************* */
// Check AdjointTranspose and jacobians
TEST(Pose3, AdjointTranspose)
{
  Vector6 xi = (Vector6() << 0.1, 1.2, 2.3, 3.1, 1.4, 4.5).finished();

  // Check evaluation
  EQUALITY(static_cast<Vector>(T.AdjointMap().transpose() * xi),
           T.AdjointTranspose(xi));
  EQUALITY(static_cast<Vector>(T2.AdjointMap().transpose() * xi),
           T2.AdjointTranspose(xi));
  EQUALITY(static_cast<Vector>(T3.AdjointMap().transpose() * xi),
           T3.AdjointTranspose(xi));

  // Check jacobians
  Matrix6 actualH1, actualH2, expectedH1, expectedH2;
  std::function<Vector6(const Pose3&, const Vector6&)> AdjointTranspose_proxy =
      [&](const Pose3& T, const Vector6& xi) {
        return T.AdjointTranspose(xi);
      };

  T.AdjointTranspose(xi, actualH1, actualH2);
  expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T, xi);
  expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T, xi);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2));

  T2.AdjointTranspose(xi, actualH1, actualH2);
  expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T2, xi);
  expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T2, xi);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2));

  T3.AdjointTranspose(xi, actualH1, actualH2);
  expectedH1 = numericalDerivative21(AdjointTranspose_proxy, T3, xi);
  expectedH2 = numericalDerivative22(AdjointTranspose_proxy, T3, xi);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2));
}

/* ************************************************************************* */
// assert that T*wedge(xi)*T^-1 is equal to wedge(Ad_T(xi))
TEST(Pose3, Adjoint_hat)
{
  auto hat = [](const Vector& xi) { return ::wedge<Pose3>(xi); };
  Matrix4 expected = T.matrix() * hat(screwPose3::xi) * T.matrix().inverse();
  Matrix4 xiprime = hat(T.Adjoint(screwPose3::xi));
  EXPECT(assert_equal(expected, xiprime, 1e-6));

  Matrix4 expected2 = T2.matrix() * hat(screwPose3::xi) * T2.matrix().inverse();
  Matrix4 xiprime2 = hat(T2.Adjoint(screwPose3::xi));
  EXPECT(assert_equal(expected2, xiprime2, 1e-6));

  Matrix4 expected3 = T3.matrix() * hat(screwPose3::xi) * T3.matrix().inverse();
  Matrix4 xiprime3 = hat(T3.Adjoint(screwPose3::xi));
  EXPECT(assert_equal(expected3, xiprime3, 1e-6));
}

/* ************************************************************************* */
/** Agrawal06iros version of exponential map */
Pose3 Agrawal06iros(const Vector& xi) {
  Vector w = xi.head(3);
  Vector v = xi.tail(3);
  double t = w.norm();
  if (t < 1e-5)
    return Pose3(Rot3(), Point3(v));
  else {
    Matrix W = skewSymmetric(w/t);
    Matrix A = I_3x3 + ((1 - cos(t)) / t) * W + ((t - sin(t)) / t) * (W * W);
    return Pose3(Rot3::Expmap (w), Point3(A * v));
  }
}

/* ************************************************************************* */
TEST(Pose3, expmaps_galore_full)
{
  Vector xi; Pose3 actual;
  xi = (Vector(6) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
  actual = Pose3::Expmap(xi);
  EXPECT(assert_equal(expm<Pose3>(xi), actual,1e-6));
  EXPECT(assert_equal(Agrawal06iros(xi), actual,1e-6));
  EXPECT(assert_equal(xi, Pose3::Logmap(actual),1e-6));

  xi = (Vector(6) << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6).finished();
  for (double theta=1.0;0.3*theta<=M_PI;theta*=2) {
    Vector txi = xi*theta;
    actual = Pose3::Expmap(txi);
    EXPECT(assert_equal(expm<Pose3>(txi,30), actual,1e-6));
    EXPECT(assert_equal(Agrawal06iros(txi), actual,1e-6));
    Vector log = Pose3::Logmap(actual);
    EXPECT(assert_equal(actual, Pose3::Expmap(log),1e-6));
    EXPECT(assert_equal(txi,log,1e-6)); // not true once wraps
  }

  // Works with large v as well, but expm needs 10 iterations!
  xi = (Vector(6) << 0.2, 0.3, -0.8, 100.0, 120.0, -60.0).finished();
  actual = Pose3::Expmap(xi);
  EXPECT(assert_equal(expm<Pose3>(xi,10), actual,1e-5));
  EXPECT(assert_equal(Agrawal06iros(xi), actual,1e-9));
  EXPECT(assert_equal(xi, Pose3::Logmap(actual),1e-9));
}

/* ************************************************************************* */
// Check translation and its pushforward
TEST(Pose3, translation) {
  Matrix actualH;
  EXPECT(assert_equal(Point3(3.5, -8.2, 4.2), T.translation(actualH), 1e-8));

  Matrix numericalH = numericalDerivative11<Point3, Pose3>(
      std::bind(&Pose3::translation, std::placeholders::_1, boost::none), T);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

/* ************************************************************************* */
// Check rotation and its pushforward
TEST(Pose3, rotation) {
  Matrix actualH;
  EXPECT(assert_equal(R, T.rotation(actualH), 1e-8));

  Matrix numericalH = numericalDerivative11<Rot3, Pose3>(
      std::bind(&Pose3::rotation, std::placeholders::_1, boost::none), T);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

/* ************************************************************************* */
TEST(Pose3, Adjoint_compose_full)
{
  // To debug derivatives of compose, assert that
  // T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
  const Pose3& T1 = T;
  Vector x = (Vector(6) << 0.1, 0.1, 0.1, 0.4, 0.2, 0.8).finished();
  Pose3 expected = T1 * Pose3::Expmap(x) * T2;
  Vector y = T2.inverse().Adjoint(x);
  Pose3 actual = T1 * T2 * Pose3::Expmap(y);
  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* ************************************************************************* */
// Check compose and its pushforward
// NOTE: testing::compose<Pose3>(t1,t2) = t1.compose(t2)  (see lieProxies.h)
TEST( Pose3, compose )
{
  Matrix actual = (T2*T2).matrix();
  Matrix expected = T2.matrix()*T2.matrix();
  EXPECT(assert_equal(actual,expected,1e-8));

  Matrix actualDcompose1, actualDcompose2;
  T2.compose(T2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T2, T2);
  EXPECT(assert_equal(numericalH1,actualDcompose1,5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T2, T2);
  EXPECT(assert_equal(numericalH2,actualDcompose2,1e-4));
}

/* ************************************************************************* */
// Check compose and its pushforward, another case
TEST( Pose3, compose2 )
{
  const Pose3& T1 = T;
  Matrix actual = (T1*T2).matrix();
  Matrix expected = T1.matrix()*T2.matrix();
  EXPECT(assert_equal(actual,expected,1e-8));

  Matrix actualDcompose1, actualDcompose2;
  T1.compose(T2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T1, T2);
  EXPECT(assert_equal(numericalH1,actualDcompose1,5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T1, T2);
  EXPECT(assert_equal(numericalH2,actualDcompose2,1e-5));
}

/* ************************************************************************* */
TEST( Pose3, inverse)
{
  Matrix actualDinverse;
  Matrix actual = T.inverse(actualDinverse).matrix();
  Matrix expected = T.matrix().inverse();
  EXPECT(assert_equal(actual,expected,1e-8));

  Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
  EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
  EXPECT(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));
}

/* ************************************************************************* */
TEST( Pose3, inverseDerivatives2)
{
  Rot3 R = Rot3::Rodrigues(0.3,0.4,-0.5);
  Point3 t(3.5,-8.2,4.2);
  Pose3 T(R,t);

  Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
  Matrix actualDinverse;
  T.inverse(actualDinverse);
  EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
  EXPECT(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));
}

/* ************************************************************************* */
TEST( Pose3, compose_inverse)
{
  Matrix actual = (T*T.inverse()).matrix();
  Matrix expected = I_4x4;
  EXPECT(assert_equal(actual,expected,1e-8));
}

/* ************************************************************************* */
Point3 transformFrom_(const Pose3& pose, const Point3& point) {
  return pose.transformFrom(point);
}
TEST(Pose3, Dtransform_from1_a) {
  Matrix actualDtransform_from1;
  T.transformFrom(P, actualDtransform_from1, boost::none);
  Matrix numerical = numericalDerivative21(transformFrom_, T, P);
  EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));
}

TEST(Pose3, Dtransform_from1_b) {
  Pose3 origin;
  Matrix actualDtransform_from1;
  origin.transformFrom(P, actualDtransform_from1, boost::none);
  Matrix numerical = numericalDerivative21(transformFrom_, origin, P);
  EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));
}

TEST(Pose3, Dtransform_from1_c) {
  Point3 origin(0, 0, 0);
  Pose3 T0(R, origin);
  Matrix actualDtransform_from1;
  T0.transformFrom(P, actualDtransform_from1, boost::none);
  Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
  EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));
}

TEST(Pose3, Dtransform_from1_d) {
  Rot3 I;
  Point3 t0(100, 0, 0);
  Pose3 T0(I, t0);
  Matrix actualDtransform_from1;
  T0.transformFrom(P, actualDtransform_from1, boost::none);
  // print(computed, "Dtransform_from1_d computed:");
  Matrix numerical = numericalDerivative21(transformFrom_, T0, P);
  // print(numerical, "Dtransform_from1_d numerical:");
  EXPECT(assert_equal(numerical, actualDtransform_from1, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, Dtransform_from2) {
  Matrix actualDtransform_from2;
  T.transformFrom(P, boost::none, actualDtransform_from2);
  Matrix numerical = numericalDerivative22(transformFrom_, T, P);
  EXPECT(assert_equal(numerical, actualDtransform_from2, 1e-8));
}

/* ************************************************************************* */
Point3 transform_to_(const Pose3& pose, const Point3& point) {
  return pose.transformTo(point);
}
TEST(Pose3, Dtransform_to1) {
  Matrix computed;
  T.transformTo(P, computed, boost::none);
  Matrix numerical = numericalDerivative21(transform_to_, T, P);
  EXPECT(assert_equal(numerical, computed, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, Dtransform_to2) {
  Matrix computed;
  T.transformTo(P, boost::none, computed);
  Matrix numerical = numericalDerivative22(transform_to_, T, P);
  EXPECT(assert_equal(numerical, computed, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, transform_to_with_derivatives) {
  Matrix actH1, actH2;
  T.transformTo(P, actH1, actH2);
  Matrix expH1 = numericalDerivative21(transform_to_, T, P),
         expH2 = numericalDerivative22(transform_to_, T, P);
  EXPECT(assert_equal(expH1, actH1, 1e-8));
  EXPECT(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, transform_from_with_derivatives) {
  Matrix actH1, actH2;
  T.transformFrom(P, actH1, actH2);
  Matrix expH1 = numericalDerivative21(transformFrom_, T, P),
         expH2 = numericalDerivative22(transformFrom_, T, P);
  EXPECT(assert_equal(expH1, actH1, 1e-8));
  EXPECT(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, transform_to_translate) {
  Point3 actual =
      Pose3(Rot3(), Point3(1, 2, 3)).transformTo(Point3(10., 20., 30.));
  Point3 expected(9., 18., 27.);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose3, transform_to_rotate) {
  Pose3 transform(Rot3::Rodrigues(0, 0, -1.570796), Point3(0, 0, 0));
  Point3 actual = transform.transformTo(Point3(2, 1, 10));
  Point3 expected(-1, 2, 10);
  EXPECT(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
// Check transformPoseFrom and its pushforward
Pose3 transformPoseFrom_(const Pose3& wTa, const Pose3& aTb) {
  return wTa.transformPoseFrom(aTb);
}

TEST(Pose3, transformPoseFrom)
{
  Matrix actual = (T2*T2).matrix();
  Matrix expected = T2.matrix()*T2.matrix();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix H1, H2;
  T2.transformPoseFrom(T2, H1, H2);

  Matrix numericalH1 = numericalDerivative21(transformPoseFrom_, T2, T2);
  EXPECT(assert_equal(numericalH1, H1, 5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(), H1, 5e-3));

  Matrix numericalH2 = numericalDerivative22(transformPoseFrom_, T2, T2);
  EXPECT(assert_equal(numericalH2, H2, 1e-4));
}

/* ************************************************************************* */
TEST(Pose3, transformTo) {
  Pose3 transform(Rot3::Rodrigues(0, 0, -1.570796), Point3(2, 4, 0));
  Point3 actual = transform.transformTo(Point3(3, 2, 10));
  Point3 expected(2, 1, 10);
  EXPECT(assert_equal(expected, actual, 0.001));
}

Pose3 transformPoseTo_(const Pose3& pose, const Pose3& pose2) {
  return pose.transformPoseTo(pose2);
}

/* ************************************************************************* */
TEST(Pose3, transformPoseTo) {
  Pose3 origin = T.transformPoseTo(T);
  EXPECT(assert_equal(Pose3{}, origin));
}

/* ************************************************************************* */
TEST(Pose3, transformPoseTo_with_derivatives) {
  Matrix actH1, actH2;
  Pose3 res = T.transformPoseTo(T2, actH1, actH2);
  EXPECT(assert_equal(res, T.inverse().compose(T2)));

  Matrix expH1 = numericalDerivative21(transformPoseTo_, T, T2),
         expH2 = numericalDerivative22(transformPoseTo_, T, T2);
  EXPECT(assert_equal(expH1, actH1, 1e-8));
  EXPECT(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, transformPoseTo_with_derivatives2) {
  Matrix actH1, actH2;
  Pose3 res = T.transformPoseTo(T3, actH1, actH2);
  EXPECT(assert_equal(res, T.inverse().compose(T3)));

  Matrix expH1 = numericalDerivative21(transformPoseTo_, T, T3),
         expH2 = numericalDerivative22(transformPoseTo_, T, T3);
  EXPECT(assert_equal(expH1, actH1, 1e-8));
  EXPECT(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST(Pose3, transformFrom) {
  Point3 actual = T3.transformFrom(Point3(0, 0, 0));
  Point3 expected = Point3(1., 2., 3.);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose3, transform_roundtrip) {
  Point3 actual = T3.transformFrom(T3.transformTo(Point3(12., -0.11, 7.0)));
  Point3 expected(12., -0.11, 7.0);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose3, Retract_LocalCoordinates)
{
  Vector6 d;
  d << 1,2,3,4,5,6; d/=10;
  const Rot3 R = Rot3::Retract(d.head<3>());
  Pose3 t = Pose3::Retract(d);
  EXPECT(assert_equal(d, Pose3::LocalCoordinates(t)));
}
/* ************************************************************************* */
TEST(Pose3, retract_localCoordinates)
{
  Vector6 d12;
  d12 << 1,2,3,4,5,6; d12/=10;
  Pose3 t1 = T, t2 = t1.retract(d12);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2)));
}
/* ************************************************************************* */
TEST(Pose3, expmap_logmap)
{
  Vector d12 = Vector6::Constant(0.1);
  Pose3 t1 = T, t2 = t1.expmap(d12);
  EXPECT(assert_equal(d12, t1.logmap(t2)));
}

/* ************************************************************************* */
TEST(Pose3, retract_localCoordinates2)
{
  Pose3 t1 = T;
  Pose3 t2 = T3;
  Pose3 origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
  // TODO(hayk): This currently fails!
  // EXPECT(assert_equal(d12, -d21));
}
/* ************************************************************************* */
TEST(Pose3, manifold_expmap)
{
  Pose3 t1 = T;
  Pose3 t2 = T3;
  Pose3 origin;
  Vector d12 = t1.logmap(t2);
  EXPECT(assert_equal(t2, t1.expmap(d12)));
  Vector d21 = t2.logmap(t1);
  EXPECT(assert_equal(t1, t2.expmap(d21)));

  // Check that log(t1,t2)=-log(t2,t1)
  EXPECT(assert_equal(d12,-d21));
}

/* ************************************************************************* */
TEST(Pose3, subgroups)
{
  // Frank - Below only works for correct "Agrawal06iros style expmap
  // lines in canonical coordinates correspond to Abelian subgroups in SE(3)
   Vector d = (Vector(6) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
  // exp(-d)=inverse(exp(d))
   EXPECT(assert_equal(Pose3::Expmap(-d),Pose3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
   Pose3 T2 = Pose3::Expmap(2*d);
   Pose3 T3 = Pose3::Expmap(3*d);
   Pose3 T5 = Pose3::Expmap(5*d);
   EXPECT(assert_equal(T5,T2*T3));
   EXPECT(assert_equal(T5,T3*T2));
}

/* ************************************************************************* */
TEST( Pose3, between )
{
  Pose3 expected = T2.inverse() * T3;
  Matrix actualDBetween1,actualDBetween2;
  Pose3 actual = T2.between(T3, actualDBetween1,actualDBetween2);
  EXPECT(assert_equal(expected,actual));

  Matrix numericalH1 = numericalDerivative21(testing::between<Pose3> , T2, T3);
  EXPECT(assert_equal(numericalH1,actualDBetween1,5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::between<Pose3> , T2, T3);
  EXPECT(assert_equal(numericalH2,actualDBetween2,1e-5));
}

/* ************************************************************************* */
// some shared test values - pulled from equivalent test in Pose2
Point3 l1(1, 0, 0), l2(1, 1, 0), l3(2, 2, 0), l4(1, 4,-4);
Pose3 x1, x2(Rot3::Ypr(0.0, 0.0, 0.0), l2), x3(Rot3::Ypr(M_PI/4.0, 0.0, 0.0), l2);
Pose3
    xl1(Rot3::Ypr(0.0, 0.0, 0.0), Point3(1, 0, 0)),
    xl2(Rot3::Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0)),
    xl3(Rot3::Ypr(1.0, 0.0, 0.0), Point3(2, 2, 0)),
    xl4(Rot3::Ypr(0.0, 0.0, 1.0), Point3(1, 4,-4));

/* ************************************************************************* */
double range_proxy(const Pose3& pose, const Point3& point) {
  return pose.range(point);
}
TEST( Pose3, range )
{
  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish range is indeed zero
  EXPECT_DOUBLES_EQUAL(1,x1.range(l1),1e-9);

  // establish range is indeed sqrt2
  EXPECT_DOUBLES_EQUAL(sqrt(2.0),x1.range(l2),1e-9);

  // Another pair
  double actual23 = x2.range(l3, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(sqrt(2.0),actual23,1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(range_proxy, x2, l3);
  expectedH2 = numericalDerivative22(range_proxy, x2, l3);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));

  // Another test
  double actual34 = x3.range(l4, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(5,actual34,1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(range_proxy, x3, l4);
  expectedH2 = numericalDerivative22(range_proxy, x3, l4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */
double range_pose_proxy(const Pose3& pose, const Pose3& point) {
  return pose.range(point);
}
TEST( Pose3, range_pose )
{
  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish range is indeed zero
  EXPECT_DOUBLES_EQUAL(1,x1.range(xl1),1e-9);

  // establish range is indeed sqrt2
  EXPECT_DOUBLES_EQUAL(sqrt(2.0),x1.range(xl2),1e-9);

  // Another pair
  double actual23 = x2.range(xl3, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(sqrt(2.0),actual23,1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(range_pose_proxy, x2, xl3);
  expectedH2 = numericalDerivative22(range_pose_proxy, x2, xl3);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));

  // Another test
  double actual34 = x3.range(xl4, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(5,actual34,1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(range_pose_proxy, x3, xl4);
  expectedH2 = numericalDerivative22(range_pose_proxy, x3, xl4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */
Unit3 bearing_proxy(const Pose3& pose, const Point3& point) {
  return pose.bearing(point);
}
TEST(Pose3, Bearing) {
  Matrix expectedH1, actualH1, expectedH2, actualH2;
  EXPECT(assert_equal(Unit3(1, 0, 0), x1.bearing(l1, actualH1, actualH2), 1e-9));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_proxy, x1, l1);
  expectedH2 = numericalDerivative22(bearing_proxy, x1, l1);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-5));
}

TEST(Pose3, Bearing2) {
  Matrix expectedH1, actualH1, expectedH2, actualH2;
  EXPECT(assert_equal(Unit3(0,0.6,-0.8), x2.bearing(l4, actualH1, actualH2), 1e-9));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_proxy, x2, l4);
  expectedH2 = numericalDerivative22(bearing_proxy, x2, l4);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-5));
}

TEST(Pose3, PoseToPoseBearing) {
  Matrix expectedH1, actualH1, expectedH2, actualH2, H2block;
  EXPECT(assert_equal(Unit3(0,1,0), xl1.bearing(xl2, actualH1, actualH2), 1e-9));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_proxy, xl1, l2);

  // Since the second pose is treated as a point, the value calculated by
  // numericalDerivative22 only depends on the position of the pose. Here, we
  // calculate the Jacobian w.r.t. the second pose's position, and then augment
  // that with zeroes in the block that is w.r.t. the second pose's
  // orientation.
  H2block = numericalDerivative22(bearing_proxy, xl1, l2);
  expectedH2 = Matrix(2, 6);
  expectedH2.setZero();
  expectedH2.block<2, 3>(0, 3) = H2block;

  EXPECT(assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-5));
}

/* ************************************************************************* */
TEST( Pose3, unicycle )
{
  // velocity in X should be X in inertial frame, rather than global frame
  Vector x_step = Vector::Unit(6,3)*1.0;
  EXPECT(assert_equal(Pose3(Rot3::Ypr(0,0,0), l1), expmap_default<Pose3>(x1, x_step), tol));
  EXPECT(assert_equal(Pose3(Rot3::Ypr(0,0,0), Point3(2,1,0)), expmap_default<Pose3>(x2, x_step), tol));
  EXPECT(assert_equal(Pose3(Rot3::Ypr(M_PI/4.0,0,0), Point3(2,2,0)), expmap_default<Pose3>(x3, sqrt(2.0) * x_step), tol));
}

/* ************************************************************************* */
TEST( Pose3, adjointMap) {
  Matrix res = Pose3::adjointMap(screwPose3::xi);
  Matrix wh = skewSymmetric(screwPose3::xi(0), screwPose3::xi(1), screwPose3::xi(2));
  Matrix vh = skewSymmetric(screwPose3::xi(3), screwPose3::xi(4), screwPose3::xi(5));
  Matrix6 expected;
  expected << wh, Z_3x3, vh, wh;
  EXPECT(assert_equal(expected,res,1e-5));
}

/* ************************************************************************* */
TEST(Pose3, Align1) {
  Pose3 expected(Rot3(), Point3(10,10,0));

  vector<Point3Pair> correspondences;
  Point3Pair ab1(make_pair(Point3(10,10,0), Point3(0,0,0)));
  Point3Pair ab2(make_pair(Point3(30,20,0), Point3(20,10,0)));
  Point3Pair ab3(make_pair(Point3(20,30,0), Point3(10,20,0)));
  correspondences += ab1, ab2, ab3;

  boost::optional<Pose3> actual = Pose3::Align(correspondences);
  EXPECT(assert_equal(expected, *actual));
}

/* ************************************************************************* */
TEST(Pose3, Align2) {
  Point3 t(20,10,5);
  Rot3 R = Rot3::RzRyRx(0.3, 0.2, 0.1);
  Pose3 expected(R, t);

  vector<Point3Pair> correspondences;
  Point3 p1(0,0,1), p2(10,0,2), p3(20,-10,30);
  Point3 q1 = expected.transformFrom(p1),
         q2 = expected.transformFrom(p2),
         q3 = expected.transformFrom(p3);
  Point3Pair ab1(make_pair(q1, p1));
  Point3Pair ab2(make_pair(q2, p2));
  Point3Pair ab3(make_pair(q3, p3));
  correspondences += ab1, ab2, ab3;

  boost::optional<Pose3> actual = Pose3::Align(correspondences);
  EXPECT(assert_equal(expected, *actual, 1e-5));
}

/* ************************************************************************* */
TEST( Pose3, ExpmapDerivative1) {
  Matrix6 actualH;
  Vector6 w; w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0;
  Pose3::Expmap(w,actualH);
  Matrix expectedH = numericalDerivative21<Pose3, Vector6,
      OptionalJacobian<6, 6> >(&Pose3::Expmap, w, boost::none);
  EXPECT(assert_equal(expectedH, actualH));
}

/* ************************************************************************* */
TEST(Pose3, ExpmapDerivative2) {
  // Iserles05an (Lie-group Methods) says:
  // scalar is easy: d exp(a(t)) / dt = exp(a(t)) a'(t)
  // matrix is hard: d exp(A(t)) / dt = exp(A(t)) dexp[-A(t)] A'(t)
  // where A(t): T -> se(3) is a trajectory in the tangent space of SE(3)
  // and dexp[A] is a linear map from 4*4 to 4*4 derivatives of se(3)
  // Hence, the above matrix equation is typed: 4*4 = SE(3) * linear_map(4*4)

  // In GTSAM, we don't work with the Lie-algebra elements A directly, but with 6-vectors.
  // xi is easy: d Expmap(xi(t)) / dt = ExmapDerivative[xi(t)] * xi'(t)

  // Let's verify the above formula.

  auto xi = [](double t) {
    Vector6 v;
    v << 2 * t, sin(t), 4 * t * t, 2 * t, sin(t), 4 * t * t;
    return v;
  };
  auto xi_dot = [](double t) {
    Vector6 v;
    v << 2, cos(t), 8 * t, 2, cos(t), 8 * t;
    return v;
  };

  // We define a function T
  auto T = [xi](double t) { return Pose3::Expmap(xi(t)); };

  for (double t = -2.0; t < 2.0; t += 0.3) {
    const Matrix expected = numericalDerivative11<Pose3, double>(T, t);
    const Matrix actual = Pose3::ExpmapDerivative(xi(t)) * xi_dot(t);
    CHECK(assert_equal(expected, actual, 1e-7));
  }
}

TEST( Pose3, ExpmapDerivativeQr) {
  Vector6 w = Vector6::Random();
  w.head<3>().normalize();
  w.head<3>() = w.head<3>() * 0.9e-2;
  Matrix3 actualQr = Pose3::ComputeQforExpmapDerivative(w, 0.01);
  Matrix expectedH = numericalDerivative21<Pose3, Vector6,
      OptionalJacobian<6, 6> >(&Pose3::Expmap, w, boost::none);
  Matrix3 expectedQr = expectedH.bottomLeftCorner<3, 3>();
  EXPECT(assert_equal(expectedQr, actualQr, 1e-6));
}

/* ************************************************************************* */
TEST( Pose3, LogmapDerivative) {
  Matrix6 actualH;
  Vector6 w; w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0;
  Pose3 p = Pose3::Expmap(w);
  EXPECT(assert_equal(w, Pose3::Logmap(p,actualH), 1e-5));
  Matrix expectedH = numericalDerivative21<Vector6, Pose3,
      OptionalJacobian<6, 6> >(&Pose3::Logmap, p, boost::none);
  EXPECT(assert_equal(expectedH, actualH));
}

/* ************************************************************************* */
Vector6 testDerivAdjoint(const Vector6& xi, const Vector6& v) {
  return Pose3::adjointMap(xi) * v;
}

TEST( Pose3, adjoint) {
  Vector6 v = (Vector6() << 1, 2, 3, 4, 5, 6).finished();
  Vector expected = testDerivAdjoint(screwPose3::xi, v);

  Matrix actualH1, actualH2;
  Vector actual = Pose3::adjoint(screwPose3::xi, v, actualH1, actualH2);

  Matrix numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
      testDerivAdjoint, screwPose3::xi, v, 1e-5);
  Matrix numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
      testDerivAdjoint, screwPose3::xi, v, 1e-5);

  EXPECT(assert_equal(expected,actual,1e-5));
  EXPECT(assert_equal(numericalH1,actualH1,1e-5));
  EXPECT(assert_equal(numericalH2,actualH2,1e-5));
}

/* ************************************************************************* */
Vector6 testDerivAdjointTranspose(const Vector6& xi, const Vector6& v) {
  return Pose3::adjointMap(xi).transpose() * v;
}

TEST( Pose3, adjointTranspose) {
  Vector xi = (Vector(6) << 0.01, 0.02, 0.03, 1.0, 2.0, 3.0).finished();
  Vector v = (Vector(6) << 0.04, 0.05, 0.06, 4.0, 5.0, 6.0).finished();
  Vector expected = testDerivAdjointTranspose(xi, v);

  Matrix actualH1, actualH2;
  Vector actual = Pose3::adjointTranspose(xi, v, actualH1, actualH2);

  Matrix numericalH1 = numericalDerivative21<Vector6, Vector6, Vector6>(
      testDerivAdjointTranspose, xi, v, 1e-5);
  Matrix numericalH2 = numericalDerivative22<Vector6, Vector6, Vector6>(
      testDerivAdjointTranspose, xi, v, 1e-5);

  EXPECT(assert_equal(expected,actual,1e-15));
  EXPECT(assert_equal(numericalH1,actualH1,1e-5));
  EXPECT(assert_equal(numericalH2,actualH2,1e-5));
}

/* ************************************************************************* */
TEST( Pose3, stream) {
  std::ostringstream os;
  os << Pose3();

  string expected = "R: [\n\t1, 0, 0;\n\t0, 1, 0;\n\t0, 0, 1\n]\nt: 0 0 0";
  EXPECT(os.str() == expected);
}

//******************************************************************************
TEST(Pose3 , Invariants) {
  Pose3 id;

  EXPECT(check_group_invariants(id,id));
  EXPECT(check_group_invariants(id,T3));
  EXPECT(check_group_invariants(T2,id));
  EXPECT(check_group_invariants(T2,T3));

  EXPECT(check_manifold_invariants(id,id));
  EXPECT(check_manifold_invariants(id,T3));
  EXPECT(check_manifold_invariants(T2,id));
  EXPECT(check_manifold_invariants(T2,T3));
}

//******************************************************************************
TEST(Pose3 , LieGroupDerivatives) {
  Pose3 id;

  CHECK_LIE_GROUP_DERIVATIVES(id,id);
  CHECK_LIE_GROUP_DERIVATIVES(id,T2);
  CHECK_LIE_GROUP_DERIVATIVES(T2,id);
  CHECK_LIE_GROUP_DERIVATIVES(T2,T3);
}

//******************************************************************************
TEST(Pose3 , ChartDerivatives) {
  Pose3 id;
  if (ROT3_DEFAULT_COORDINATES_MODE == Rot3::EXPMAP) {
    CHECK_CHART_DERIVATIVES(id,id);
    CHECK_CHART_DERIVATIVES(id,T2);
    CHECK_CHART_DERIVATIVES(T2,id);
    CHECK_CHART_DERIVATIVES(T2,T3);
  }
}

//******************************************************************************
#include "testPoseAdjointMap.h"

TEST(Pose3, TransformCovariance6MapTo2d) {
  // Create 3d scenarios that map to 2d configurations and compare with Pose2 results.
  using namespace test_pose_adjoint_map;

  Vector3 s2{0.1, 0.3, 0.7};
  Pose2 p2{1.1, 1.5, 31. * degree};
  auto cov2 = FullCovarianceFromSigmas<Pose2>(s2);
  auto transformed2 = TransformCovariance<Pose2>{p2}(cov2);

  auto match_cov3_to_cov2 = [&](int spatial_axis0, int spatial_axis1, int r_axis,
                                const Pose2::Jacobian &cov2, const Pose3::Jacobian &cov3) -> void
  {
    EXPECT(assert_equal(
      Vector3{cov2.diagonal()},
      Vector3{cov3(spatial_axis0, spatial_axis0), cov3(spatial_axis1, spatial_axis1), cov3(r_axis, r_axis)}));
    EXPECT(assert_equal(
      Vector3{cov2(1, 0), cov2(2, 0), cov2(2, 1)},
      Vector3{cov3(spatial_axis1, spatial_axis0), cov3(r_axis, spatial_axis0), cov3(r_axis, spatial_axis1)}));
  };

  // rotate around x axis
  {
    auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << s2(2), 0., 0., 0., s2(0), s2(1)).finished());
    auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(p2.theta(), 0., 0.), {0., p2.x(), p2.y()}}}(cov3);
    match_cov3_to_cov2(4, 5, 0, transformed2, transformed3);
  }

  // rotate around y axis
  {
    auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0., s2(2), 0., s2(1), 0., s2(0)).finished());
    auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., p2.theta(), 0.), {p2.y(), 0., p2.x()}}}(cov3);
    match_cov3_to_cov2(5, 3, 1, transformed2, transformed3);
  }

  // rotate around z axis
  {
    auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0., 0., s2(2), s2(0), s2(1), 0.).finished());
    auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 0., p2.theta()), {p2.x(), p2.y(), 0.}}}(cov3);
    match_cov3_to_cov2(3, 4, 2, transformed2, transformed3);
  }
}

/* ************************************************************************* */
TEST(Pose3, TransformCovariance6) {
  // Use simple covariance matrices and transforms to create tests that can be
  // validated with simple computations.
  using namespace test_pose_adjoint_map;

  // rotate 90 around z axis and then 90 around y axis
  {
    auto cov = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0.1, 0.2, 0.3, 0.5, 0.7, 1.1).finished());
    auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 90 * degree, 90 * degree), {0., 0., 0.}}}(cov);
    // x from y, y from z, z from x
    EXPECT(assert_equal(
      (Vector6{} << cov(1, 1), cov(2, 2), cov(0, 0), cov(4, 4), cov(5, 5), cov(3, 3)).finished(),
      Vector6{transformed.diagonal()}));
    // Both the x and z axes are pointing in the negative direction.
    EXPECT(assert_equal(
      (Vector5{} << -cov(2, 1), cov(0, 1), cov(4, 1), -cov(5, 1), cov(3, 1)).finished(),
      (Vector5{} << transformed(1, 0), transformed(2, 0), transformed(3, 0),
        transformed(4, 0), transformed(5, 0)).finished()));
  }

  // translate along the x axis with uncertainty in roty and rotz
  {
    auto cov = TwoVariableCovarianceFromSigmas<Pose3>(1, 2, 0.7, 0.3);
    auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 0., 0.), {20., 0., 0.}}}(cov);
    // The uncertainty in roty and rotz causes off-diagonal covariances
    EXPECT(assert_equal(0.7 * 0.7 * 20., transformed(5, 1)));
    EXPECT(assert_equal(0.7 * 0.7 * 20. * 20., transformed(5, 5)));
    EXPECT(assert_equal(-0.3 * 0.3 * 20., transformed(4, 2)));
    EXPECT(assert_equal(0.3 * 0.3 * 20. * 20., transformed(4, 4)));
    EXPECT(assert_equal(-0.3 * 0.7 * 20., transformed(4, 1)));
    EXPECT(assert_equal(0.3 * 0.7 * 20., transformed(5, 2)));
    EXPECT(assert_equal(-0.3 * 0.7 * 20. * 20., transformed(5, 4)));
  }

  // rotate around x axis and translate along the x axis with uncertainty in rotx
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose3>(0, 0.1);
    auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(90 * degree, 0., 0.), {20., 0., 0.}}}(cov);
    // No change
    EXPECT(assert_equal(cov, transformed));
  }

  // rotate around x axis and translate along the x axis with uncertainty in roty
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose3>(1, 0.1);
    auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(90 * degree, 0., 0.), {20., 0., 0.}}}(cov);
    // Uncertainty is spread to other dimensions.
    EXPECT(assert_equal(
      (Vector6{} << 0., 0., 0.1 * 0.1, 0., 0.1 * 0.1 * 20. * 20., 0.).finished(),
      Vector6{transformed.diagonal()}));
  }
}

/* ************************************************************************* */
TEST(Pose3, interpolate) {
  EXPECT(assert_equal(T2, interpolate(T2,T3, 0.0)));
  EXPECT(assert_equal(T3, interpolate(T2,T3, 1.0)));

  // Trivial example: start at origin and move to (1, 0, 0) while rotating pi/2
  // about z-axis.
  Pose3 start;
  Pose3 end(Rot3::Rz(M_PI_2), Point3(1, 0, 0));
  // This interpolation is easy to calculate by hand.
  double t = 0.5;
  Pose3 expected0(Rot3::Rz(M_PI_4), Point3(0.5, 0, 0));
  EXPECT(assert_equal(expected0, start.interpolateRt(end, t)));

  // Example from Peter Corke
  // https://robotacademy.net.au/lesson/interpolating-pose-in-3d/
  t = 0.0759;  // corresponds to the 10th element when calling `ctraj` in
               // the video
  Pose3 O;
  Pose3 F(Rot3::Roll(0.6).compose(Rot3::Pitch(0.8)).compose(Rot3::Yaw(1.4)),
          Point3(1, 2, 3));

  // The expected answer matches the result presented in the video.
  Pose3 expected1(interpolate(O.rotation(), F.rotation(), t),
                  interpolate(O.translation(), F.translation(), t));
  EXPECT(assert_equal(expected1, O.interpolateRt(F, t)));

  // Non-trivial interpolation, translation value taken from output.
  Pose3 expected2(interpolate(T2.rotation(), T3.rotation(), t),
                  interpolate(T2.translation(), T3.translation(), t));
  EXPECT(assert_equal(expected2, T2.interpolateRt(T3, t)));
}

/* ************************************************************************* */
Pose3 testing_interpolate(const Pose3& t1, const Pose3& t2, double gamma) { return interpolate(t1,t2,gamma); }

TEST(Pose3, interpolateJacobians) {
  {
    Pose3 X = Pose3::Identity();
    Pose3 Y(Rot3::Rz(M_PI_2), Point3(1, 0, 0));
    double t = 0.5;
    Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0.5, -0.207107, 0)); // note: different from test above: this is full Pose3 interpolation
    Matrix actualJacobianX, actualJacobianY;
    EXPECT(assert_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

    Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

    Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
  }
  {
    Pose3 X = Pose3::Identity();
    Pose3 Y(Rot3::Identity(), Point3(1, 0, 0));
    double t = 0.3;
    Pose3 expectedPoseInterp(Rot3::Identity(), Point3(0.3, 0, 0));
    Matrix actualJacobianX, actualJacobianY;
    EXPECT(assert_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

    Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

    Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
  }
  {
    Pose3 X = Pose3::Identity();
    Pose3 Y(Rot3::Rz(M_PI_2), Point3(0, 0, 0));
    double t = 0.5;
    Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0, 0, 0));
    Matrix actualJacobianX, actualJacobianY;
    EXPECT(assert_equal(expectedPoseInterp, interpolate(X, Y, t, actualJacobianX, actualJacobianY), 1e-5));

    Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

    Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
  }
  {
    Pose3 X(Rot3::Ypr(0.1,0.2,0.3), Point3(10, 5, -2));
    Pose3 Y(Rot3::Ypr(1.1,-2.2,-0.3), Point3(-5, 1, 1));
    double t = 0.3;
    Pose3 expectedPoseInterp(Rot3::Rz(M_PI_4), Point3(0, 0, 0));
    Matrix actualJacobianX, actualJacobianY;
    interpolate(X, Y, t, actualJacobianX, actualJacobianY);

    Matrix expectedJacobianX = numericalDerivative31<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianX,actualJacobianX,1e-6));

    Matrix expectedJacobianY = numericalDerivative32<Pose3,Pose3,Pose3,double>(testing_interpolate, X, Y, t);
    EXPECT(assert_equal(expectedJacobianY,actualJacobianY,1e-6));
  }
}

/* ************************************************************************* */
TEST(Pose3, Create) {
  Matrix63 actualH1, actualH2;
  Pose3 actual = Pose3::Create(R, P2, actualH1, actualH2);
  EXPECT(assert_equal(T, actual));
  std::function<Pose3(Rot3, Point3)> create =
      std::bind(Pose3::Create, std::placeholders::_1, std::placeholders::_2,
                boost::none, boost::none);
  EXPECT(assert_equal(numericalDerivative21<Pose3,Rot3,Point3>(create, R, P2), actualH1, 1e-9));
  EXPECT(assert_equal(numericalDerivative22<Pose3,Rot3,Point3>(create, R, P2), actualH2, 1e-9));
}

/* ************************************************************************* */
TEST(Pose3, Print) {
  Pose3 pose(Rot3::Identity(), Point3(1, 2, 3));

  // Generate the expected output
  std::string expected = "R: [\n\t1, 0, 0;\n\t0, 1, 0;\n\t0, 0, 1\n]\nt: 1 2 3\n";

  EXPECT(assert_print_equal(expected, pose));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
