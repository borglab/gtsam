/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPose2.cpp
 * @brief  Unit tests for Pose2 class
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>

#include <optional>
#include <cmath>
#include <iostream>

using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(Pose2)
GTSAM_CONCEPT_LIE_INST(Pose2)

//******************************************************************************
TEST(Pose2 , Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Pose2 >);
  GTSAM_CONCEPT_ASSERT(IsManifold<Pose2 >);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Pose2 >);
}

/* ************************************************************************* */
TEST(Pose2, constructors) {
  Point2 p(0,0);
  Pose2 pose(0,p);
  Pose2 origin;
  assert_equal(pose,origin);
  Pose2 t(M_PI/2.0+0.018, Point2(1.015, 2.01));
  EXPECT(assert_equal(t,Pose2((Matrix)t.matrix())));
}

/* ************************************************************************* */
TEST(Pose2, manifold) {
  Pose2 t1(M_PI/2.0, Point2(1, 2));
  Pose2 t2(M_PI/2.0+0.018, Point2(1.015, 2.01));
  Pose2 origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  EXPECT(assert_equal(t2, t1*origin.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
  EXPECT(assert_equal(t1, t2*origin.retract(d21)));
}

/* ************************************************************************* */
TEST(Pose2, retract) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
#ifdef SLOW_BUT_CORRECT_EXPMAP
  Pose2 expected(1.00811, 2.01528, 2.5608);
#else
  Pose2 expected(M_PI/2.0+0.99, Point2(1.015, 2.01));
#endif
  Pose2 actual = pose.retract(Vector3(0.01, -0.015, 0.99));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
  Pose2 expected(1.00811, 2.01528, 2.5608);
  Pose2 actual = expmap_default<Pose2>(pose, Vector3(0.01, -0.015, 0.99));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap2) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
  Pose2 expected(1.00811, 2.01528, 2.5608);
  Pose2 actual = expmap_default<Pose2>(pose, Vector3(0.01, -0.015, 0.99));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap3) {
  // do an actual series exponential map
  // see e.g. http://www.cis.upenn.edu/~cis610/cis610lie1.ps
  Matrix A = (Matrix(3,3) <<
      0.0, -0.99,  0.01,
      0.99,  0.0, -0.015,
      0.0,   0.0,  0.0).finished();
  Matrix A2 = A*A/2.0, A3 = A2*A/3.0, A4=A3*A/4.0;
  Matrix expected = I_3x3 + A + A2 + A3 + A4;

  Vector v = Vector3(0.01, -0.015, 0.99);
  Pose2 pose = Pose2::Expmap(v);
  Pose2 pose2(v);
  EXPECT(assert_equal(pose, pose2));
  Matrix actual = pose.matrix();
  //EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose2, expmap0a) {
  Pose2 expected(0.0101345, -0.0149092, 0.018);
  Pose2 actual = Pose2::Expmap(Vector3(0.01, -0.015, 0.018));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap0b) {
  // a quarter turn
  Pose2 expected(1.0, 1.0, M_PI/2);
  Pose2 actual = Pose2::Expmap((Vector(3) << M_PI/2, 0.0, M_PI/2).finished());
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap0c) {
  // a half turn
  Pose2 expected(0.0, 2.0, M_PI);
  Pose2 actual = Pose2::Expmap((Vector(3) << M_PI, 0.0, M_PI).finished());
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap0d) {
  // a full turn
  Pose2 expected(0, 0, 0);
  Pose2 actual = Pose2::Expmap((Vector(3) << 2*M_PI, 0.0, 2*M_PI).finished());
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screwPose2 {
  double w=0.3;
  Vector xi = (Vector(3) << 0.0, w, w).finished();
  Rot2 expectedR = Rot2::fromAngle(w);
  Point2 expectedT(-0.0446635, 0.29552);
  Pose2 expected(expectedR, expectedT);
}

TEST(Pose2, expmap_c)
{
  EXPECT(assert_equal(screwPose2::expected, expm<Pose2>(screwPose2::xi),1e-6));
  EXPECT(assert_equal(screwPose2::expected, Pose2::Expmap(screwPose2::xi),1e-6));
  EXPECT(assert_equal(screwPose2::xi, Pose2::Logmap(screwPose2::expected),1e-6));
}

/* ************************************************************************* */
TEST(Pose2, expmap_c_full)
{
  double w=0.3;
  Vector xi = (Vector(3) << 0.0, w, w).finished();
  Rot2 expectedR = Rot2::fromAngle(w);
  Point2 expectedT(-0.0446635, 0.29552);
  Pose2 expected(expectedR, expectedT);
  EXPECT(assert_equal(expected, expm<Pose2>(xi),1e-6));
  EXPECT(assert_equal(expected, Pose2::Expmap(xi),1e-6));
  EXPECT(assert_equal(xi, Pose2::Logmap(expected),1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(Pose2, Adjoint_full) {
  Pose2 T(1, 2, 3);
  Pose2 expected = T * Pose2::Expmap(screwPose2::xi) * T.inverse();
  Vector xiprime = T.Adjoint(screwPose2::xi);
  EXPECT(assert_equal(expected, Pose2::Expmap(xiprime), 1e-6));

  Vector3 xi2(4, 5, 6);
  Pose2 expected2 = T * Pose2::Expmap(xi2) * T.inverse();
  Vector xiprime2 = T.Adjoint(xi2);
  EXPECT(assert_equal(expected2, Pose2::Expmap(xiprime2), 1e-6));
}

/* ************************************************************************* */
// assert that T*wedge(xi)*T^-1 is equal to wedge(Ad_T(xi))
TEST(Pose2, Adjoint_hat) {
  Pose2 T(1, 2, 3);
  auto hat = [](const Vector& xi) { return ::wedge<Pose2>(xi); };
  Matrix3 expected = T.matrix() * hat(screwPose2::xi) * T.matrix().inverse();
  Matrix3 xiprime = hat(T.Adjoint(screwPose2::xi));
  EXPECT(assert_equal(expected, xiprime, 1e-6));

  Vector3 xi2(4, 5, 6);
  Matrix3 expected2 = T.matrix() * hat(xi2) * T.matrix().inverse();
  Matrix3 xiprime2 = hat(T.Adjoint(xi2));
  EXPECT(assert_equal(expected2, xiprime2, 1e-6));
}

/* ************************************************************************* */
TEST(Pose2, logmap) {
  Pose2 pose0(M_PI/2.0, Point2(1, 2));
  Pose2 pose(M_PI/2.0+0.018, Point2(1.015, 2.01));
#ifdef SLOW_BUT_CORRECT_EXPMAP
  Vector3 expected(0.00986473, -0.0150896, 0.018);
#else
  Vector3 expected(0.01, -0.015, 0.018);
#endif
  Vector actual = pose0.localCoordinates(pose);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, logmap_full) {
  Pose2 pose0(M_PI/2.0, Point2(1, 2));
  Pose2 pose(M_PI/2.0+0.018, Point2(1.015, 2.01));
  Vector expected = Vector3(0.00986473, -0.0150896, 0.018);
  Vector actual = logmap_default<Pose2>(pose0, pose);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST( Pose2, ExpmapDerivative1) {
  Matrix3 actualH;
  Vector3 w(0.1, 0.27, -0.3);
  Pose2::Expmap(w,actualH);
  Matrix3 expectedH = numericalDerivative21<Pose2, Vector3,
      OptionalJacobian<3, 3> >(&Pose2::Expmap, w, {}, 1e-2);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

/* ************************************************************************* */
TEST( Pose2, ExpmapDerivative2) {
  Matrix3 actualH;
  Vector3 w0(0.1, 0.27, 0.0);  // alpha = 0
  Pose2::Expmap(w0,actualH);
  Matrix3 expectedH = numericalDerivative21<Pose2, Vector3,
      OptionalJacobian<3, 3> >(&Pose2::Expmap, w0, {}, 1e-2);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

/* ************************************************************************* */
TEST( Pose2, LogmapDerivative1) {
  Matrix3 actualH;
  Vector3 w(0.1, 0.27, -0.3);
  Pose2 p = Pose2::Expmap(w);
  EXPECT(assert_equal(w, Pose2::Logmap(p,actualH), 1e-5));
  Matrix3 expectedH = numericalDerivative21<Vector3, Pose2,
      OptionalJacobian<3, 3> >(&Pose2::Logmap, p, {}, 1e-2);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

/* ************************************************************************* */
TEST( Pose2, LogmapDerivative2) {
  Matrix3 actualH;
  Vector3 w0(0.1, 0.27, 0.0);  // alpha = 0
  Pose2 p = Pose2::Expmap(w0);
  EXPECT(assert_equal(w0, Pose2::Logmap(p,actualH), 1e-5));
  Matrix3 expectedH = numericalDerivative21<Vector3, Pose2,
      OptionalJacobian<3, 3> >(&Pose2::Logmap, p, {}, 1e-2);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

/* ************************************************************************* */
static Point2 transformTo_(const Pose2& pose, const Point2& point) {
  return pose.transformTo(point);
}

TEST(Pose2, transformTo) {
  Pose2 pose(M_PI / 2.0, Point2(1, 2));  // robot at (1,2) looking towards y
  Point2 point(-1, 4);                   // landmark at (-1,4)

  // expected
  Point2 expected(2, 2);
  Matrix expectedH1 =
      (Matrix(2, 3) << -1.0, 0.0, 2.0, 0.0, -1.0, -2.0).finished();
  Matrix expectedH2 = (Matrix(2, 2) << 0.0, 1.0, -1.0, 0.0).finished();

  // actual
  Matrix actualH1, actualH2;
  Point2 actual = pose.transformTo(point, actualH1, actualH2);
  EXPECT(assert_equal(expected, actual));

  EXPECT(assert_equal(expectedH1, actualH1));
  Matrix numericalH1 = numericalDerivative21(transformTo_, pose, point);
  EXPECT(assert_equal(numericalH1, actualH1));

  EXPECT(assert_equal(expectedH2, actualH2));
  Matrix numericalH2 = numericalDerivative22(transformTo_, pose, point);
  EXPECT(assert_equal(numericalH2, actualH2));
}

/* ************************************************************************* */
static Point2 transformFrom_(const Pose2& pose, const Point2& point) {
  return pose.transformFrom(point);
}

TEST(Pose2, transformFrom) {
  Pose2 pose(1., 0., M_PI / 2.0);
  Point2 pt(2., 1.);
  Matrix H1, H2;
  Point2 actual = pose.transformFrom(pt, H1, H2);

  Point2 expected(0., 2.);
  EXPECT(assert_equal(expected, actual));

  Matrix H1_expected = (Matrix(2, 3) << 0., -1., -2., 1., 0., -1.).finished();
  Matrix H2_expected = (Matrix(2, 2) << 0., -1., 1., 0.).finished();

  Matrix numericalH1 = numericalDerivative21(transformFrom_, pose, pt);
  EXPECT(assert_equal(H1_expected, H1));
  EXPECT(assert_equal(H1_expected, numericalH1));

  Matrix numericalH2 = numericalDerivative22(transformFrom_, pose, pt);
  EXPECT(assert_equal(H2_expected, H2));
  EXPECT(assert_equal(H2_expected, numericalH2));
}

/* ************************************************************************* */
TEST(Pose2, compose_a)
{
  Pose2 pose1(M_PI/4.0, Point2(sqrt(0.5), sqrt(0.5)));
  Pose2 pose2(M_PI/2.0, Point2(0.0, 2.0));

  Matrix actualDcompose1;
  Matrix actualDcompose2;
  Pose2 actual = pose1.compose(pose2, actualDcompose1, actualDcompose2);

  Pose2 expected(3.0*M_PI/4.0, Point2(-sqrt(0.5), 3.0*sqrt(0.5)));
  EXPECT(assert_equal(expected, actual));

  Matrix expectedH1 = (Matrix(3,3) <<
      0.0, 1.0, 0.0,
       -1.0, 0.0, 2.0,
      0.0, 0.0, 1.0
  ).finished();
  Matrix expectedH2 = I_3x3;
  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  EXPECT(assert_equal(expectedH1,actualDcompose1));
  EXPECT(assert_equal(numericalH1,actualDcompose1));
  EXPECT(assert_equal(expectedH2,actualDcompose2));
  EXPECT(assert_equal(numericalH2,actualDcompose2));

  Point2 point(sqrt(0.5), 3.0*sqrt(0.5));
  Point2 expected_point(-1.0, -1.0);
  Point2 actual_point1 = (pose1 * pose2).transformTo(point);
  Point2 actual_point2 = pose2.transformTo(pose1.transformTo(point));
  EXPECT(assert_equal(expected_point, actual_point1));
  EXPECT(assert_equal(expected_point, actual_point2));
}

/* ************************************************************************* */
TEST(Pose2, compose_b)
{
  Pose2 pose1(Rot2::fromAngle(M_PI/10.0), Point2(.75, .5));
  Pose2 pose2(Rot2::fromAngle(M_PI/4.0-M_PI/10.0), Point2(0.701289620636, 1.34933052585));

  Pose2 pose_expected(Rot2::fromAngle(M_PI/4.0), Point2(1.0, 2.0));

  Pose2 pose_actual_op = pose1 * pose2;
  Matrix actualDcompose1, actualDcompose2;
  Pose2 pose_actual_fcn = pose1.compose(pose2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  EXPECT(assert_equal(numericalH1,actualDcompose1,1e-5));
  EXPECT(assert_equal(numericalH2,actualDcompose2));

  EXPECT(assert_equal(pose_expected, pose_actual_op));
  EXPECT(assert_equal(pose_expected, pose_actual_fcn));
}

/* ************************************************************************* */
TEST(Pose2, compose_c)
{
  Pose2 pose1(Rot2::fromAngle(M_PI/4.0), Point2(1.0, 1.0));
  Pose2 pose2(Rot2::fromAngle(M_PI/4.0), Point2(sqrt(.5), sqrt(.5)));

  Pose2 pose_expected(Rot2::fromAngle(M_PI/2.0), Point2(1.0, 2.0));

  Pose2 pose_actual_op = pose1 * pose2;
  Matrix actualDcompose1, actualDcompose2;
  Pose2 pose_actual_fcn = pose1.compose(pose2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  EXPECT(assert_equal(numericalH1,actualDcompose1,1e-5));
  EXPECT(assert_equal(numericalH2,actualDcompose2));

  EXPECT(assert_equal(pose_expected, pose_actual_op));
  EXPECT(assert_equal(pose_expected, pose_actual_fcn));
}

/* ************************************************************************* */
TEST(Pose2, inverse )
{
  Point2 origin(0,0), t(1,2);
  Pose2 gTl(M_PI/2.0, t); // robot at (1,2) looking towards y

  Pose2 identity, lTg = gTl.inverse();
  EXPECT(assert_equal(identity,lTg.compose(gTl)));
  EXPECT(assert_equal(identity,gTl.compose(lTg)));

  Point2 l(4,5), g(-4,6);
  EXPECT(assert_equal(g,gTl*l));
  EXPECT(assert_equal(l,lTg*g));

  // Check derivative
  Matrix numericalH = numericalDerivative11<Pose2,Pose2>(testing::inverse, lTg);
  Matrix actualDinverse;
  lTg.inverse(actualDinverse);
  EXPECT(assert_equal(numericalH,actualDinverse));
}

namespace {
  /* ************************************************************************* */
  Vector homogeneous(const Point2& p) {
    return (Vector(3) << p.x(), p.y(), 1.0).finished();
  }

  /* ************************************************************************* */
  Matrix matrix(const Pose2& gTl) {
    Matrix gRl = gTl.r().matrix();
    Point2 gt = gTl.t();
    return (Matrix(3, 3) <<
      gRl(0, 0), gRl(0, 1), gt.x(),
      gRl(1, 0), gRl(1, 1), gt.y(),
      0.0,       0.0,   1.0).finished();
  }
}

/* ************************************************************************* */
TEST( Pose2, matrix )
{
  Point2 origin(0,0), t(1,2);
  Pose2 gTl(M_PI/2.0, t); // robot at (1,2) looking towards y
  Matrix gMl = matrix(gTl);
  EXPECT(assert_equal((Matrix(3,3) <<
      0.0, -1.0, 1.0,
      1.0,  0.0, 2.0,
      0.0,  0.0, 1.0).finished(),
      gMl));
  Rot2 gR1 = gTl.r();
  EXPECT(assert_equal(homogeneous(t),gMl*homogeneous(origin)));
  Point2 x_axis(1,0), y_axis(0,1);
  EXPECT(assert_equal((Matrix(2,2) <<
      0.0, -1.0,
      1.0,  0.0).finished(),
      gR1.matrix()));
  EXPECT(assert_equal(Point2(0,1),gR1*x_axis));
  EXPECT(assert_equal(Point2(-1,0),gR1*y_axis));
  EXPECT(assert_equal(homogeneous(Point2(1+0,2+1)),gMl*homogeneous(x_axis)));
  EXPECT(assert_equal(homogeneous(Point2(1-1,2+0)),gMl*homogeneous(y_axis)));

  // check inverse pose
  Matrix lMg = matrix(gTl.inverse());
  EXPECT(assert_equal((Matrix(3,3) <<
      0.0,  1.0,-2.0,
     -1.0,  0.0, 1.0,
      0.0,  0.0, 1.0).finished(),
      lMg));
}

/* ************************************************************************* */
TEST( Pose2, compose_matrix )
{
  Pose2 gT1(M_PI/2.0, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 _1T2(M_PI, Point2(-1,4));  // local robot at (-1,4) looking at negative x
  Matrix gM1(matrix(gT1)),_1M2(matrix(_1T2));
  EXPECT(assert_equal(gM1*_1M2,matrix(gT1.compose(_1T2)))); // RIGHT DOES NOT
}



/* ************************************************************************* */
TEST( Pose2, translation )  {
  Pose2 pose(3.5, -8.2, 4.2);

  Matrix actualH;
  EXPECT(assert_equal((Vector2() << 3.5, -8.2).finished(), pose.translation(actualH), 1e-8));

  std::function<Point2(const Pose2&)> f = [](const Pose2& T) { return T.translation(); };
  Matrix numericalH = numericalDerivative11<Point2, Pose2>(f, pose);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}

/* ************************************************************************* */
TEST( Pose2, rotation ) {
  Pose2 pose(3.5, -8.2, 4.2);

  Matrix actualH(4, 3);
  EXPECT(assert_equal(Rot2(4.2), pose.rotation(actualH), 1e-8));

  std::function<Rot2(const Pose2&)> f = [](const Pose2& T) { return T.rotation(); };
  Matrix numericalH = numericalDerivative11<Rot2, Pose2>(f, pose);
  EXPECT(assert_equal(numericalH, actualH, 1e-6));
}


/* ************************************************************************* */
TEST( Pose2, between )
{
  // <
  //
  //       ^
  //
  // *--0--*--*
  Pose2 gT1(M_PI/2.0, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 gT2(M_PI, Point2(-1,4));  // robot at (-1,4) looking at negative x

  Matrix actualH1,actualH2;
  Pose2 expected(M_PI/2.0, Point2(2,2));
  Pose2 actual1 = gT1.between(gT2);
  Pose2 actual2 = gT1.between(gT2,actualH1,actualH2);
  EXPECT(assert_equal(expected,actual1));
  EXPECT(assert_equal(expected,actual2));

  Matrix expectedH1 = (Matrix(3,3) <<
      0.0,-1.0,-2.0,
      1.0, 0.0,-2.0,
      0.0, 0.0,-1.0
  ).finished();
  Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(numericalH1,actualH1));
  // Assert H1 = -AdjointMap(between(p2,p1)) as in doc/math.lyx
  EXPECT(assert_equal(-gT2.between(gT1).AdjointMap(),actualH1));

  Matrix expectedH2 = (Matrix(3,3) <<
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0
  ).finished();
  Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
  EXPECT(assert_equal(expectedH2,actualH2));
  EXPECT(assert_equal(numericalH2,actualH2));

}

/* ************************************************************************* */
// reverse situation for extra test
TEST( Pose2, between2 )
{
  Pose2 p2(M_PI/2.0, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 p1(M_PI, Point2(-1,4));  // robot at (-1,4) loooking at negative x

  Matrix actualH1,actualH2;
  p1.between(p2,actualH1,actualH2);
  Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, p1, p2);
  EXPECT(assert_equal(numericalH1,actualH1));
  Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, p1, p2);
  EXPECT(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
// arbitrary, non perpendicular angles to be extra safe
TEST( Pose2, between3 )
{
  Pose2 p2(M_PI/3.0, Point2(1,2));
  Pose2 p1(M_PI/6.0, Point2(-1,4));

  Matrix actualH1,actualH2;
  p1.between(p2,actualH1,actualH2);
  Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, p1, p2);
  EXPECT(assert_equal(numericalH1,actualH1));
  Matrix numericalH2 = numericalDerivative22<Pose2,Pose2,Pose2>(testing::between, p1, p2);
  EXPECT(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST( Pose2, round_trip )
{
  Pose2 p1(1.23, 2.30, 0.2);
  Pose2 odo(0.53, 0.39, 0.15);
  Pose2 p2 = p1.compose(odo);
  EXPECT(assert_equal(odo, p1.between(p2)));
}

namespace {
  /* ************************************************************************* */
  // some shared test values
  Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI/4.0);
  Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

  /* ************************************************************************* */
  Rot2 bearing_proxy(const Pose2& pose, const Point2& pt) {
    return pose.bearing(pt);
  }
}

TEST( Pose2, bearing )
{
  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish bearing is indeed zero
  EXPECT(assert_equal(Rot2(),x1.bearing(l1)));

  // establish bearing is indeed 45 degrees
  EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),x1.bearing(l2)));

  // establish bearing is indeed 45 degrees even if shifted
  Rot2 actual23 = x2.bearing(l3, actualH1, actualH2);
  EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),actual23));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_proxy, x2, l3);
  EXPECT(assert_equal(expectedH1,actualH1));
  expectedH2 = numericalDerivative22(bearing_proxy, x2, l3);
  EXPECT(assert_equal(expectedH2,actualH2));

  // establish bearing is indeed 45 degrees even if rotated
  Rot2 actual34 = x3.bearing(l4, actualH1, actualH2);
  EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),actual34));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_proxy, x3, l4);
  expectedH2 = numericalDerivative22(bearing_proxy, x3, l4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */
namespace {
  Rot2 bearing_pose_proxy(const Pose2& pose, const Pose2& pt) {
    return pose.bearing(pt);
  }
}

TEST( Pose2, bearing_pose )
{
  Pose2 xl1(1, 0, M_PI/2.0), xl2(1, 1, M_PI), xl3(2.0, 2.0,-M_PI/2.0), xl4(1, 3, 0);

  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish bearing is indeed zero
  EXPECT(assert_equal(Rot2(),x1.bearing(xl1)));

  // establish bearing is indeed 45 degrees
  EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),x1.bearing(xl2)));

  // establish bearing is indeed 45 degrees even if shifted
  Rot2 actual23 = x2.bearing(xl3, actualH1, actualH2);
  EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),actual23));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_pose_proxy, x2, xl3);
  expectedH2 = numericalDerivative22(bearing_pose_proxy, x2, xl3);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));

  // establish bearing is indeed 45 degrees even if rotated
  Rot2 actual34 = x3.bearing(xl4, actualH1, actualH2);
  EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),actual34));

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(bearing_pose_proxy, x3, xl4);
  expectedH2 = numericalDerivative22(bearing_pose_proxy, x3, xl4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */
namespace {
  double range_proxy(const Pose2& pose, const Point2& point) {
    return pose.range(point);
  }
}
TEST( Pose2, range )
{
  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish range is indeed zero
  EXPECT_DOUBLES_EQUAL(1,x1.range(l1),1e-9);

  // establish range is indeed 45 degrees
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
  EXPECT_DOUBLES_EQUAL(2,actual34,1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(range_proxy, x3, l4);
  expectedH2 = numericalDerivative22(range_proxy, x3, l4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */
namespace {
  double range_pose_proxy(const Pose2& pose, const Pose2& point) {
    return pose.range(point);
  }
}
TEST( Pose2, range_pose )
{
  Pose2 xl1(1, 0, M_PI/2.0), xl2(1, 1, M_PI), xl3(2.0, 2.0,-M_PI/2.0), xl4(1, 3, 0);

  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish range is indeed zero
  EXPECT_DOUBLES_EQUAL(1,x1.range(xl1),1e-9);

  // establish range is indeed 45 degrees
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
  EXPECT_DOUBLES_EQUAL(2,actual34,1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(range_pose_proxy, x3, xl4);
  expectedH2 = numericalDerivative22(range_pose_proxy, x3, xl4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */

TEST(Pose2, align_1) {
  Pose2 expected(Rot2::fromAngle(0), Point2(10, 10));
  Point2Pairs ab_pairs {{Point2(10, 10), Point2(0, 0)},
                        {Point2(30, 20), Point2(20, 10)}};
  std::optional<Pose2> aTb = Pose2::Align(ab_pairs);
  EXPECT(assert_equal(expected, *aTb));
}

TEST(Pose2, align_2) {
  Point2 t(20, 10);
  Rot2 R = Rot2::fromAngle(M_PI/2.0);
  Pose2 expected(R, t);

  Point2 b1(0, 0), b2(10, 0);
  Point2Pairs ab_pairs {{expected.transformFrom(b1), b1},
                        {expected.transformFrom(b2), b2}};

  std::optional<Pose2> aTb = Pose2::Align(ab_pairs);
  EXPECT(assert_equal(expected, *aTb));
}

namespace align_3 {
  Point2 t(10, 10);
  Pose2 expected(Rot2::fromAngle(2*M_PI/3), t);
  Point2 b1(0, 0), b2(10, 0), b3(10, 10);
  Point2 a1 = expected.transformFrom(b1),
         a2 = expected.transformFrom(b2),
         a3 = expected.transformFrom(b3);
}

TEST(Pose2, align_3) {
  using namespace align_3;

  Point2Pair ab1(make_pair(a1, b1));
  Point2Pair ab2(make_pair(a2, b2));
  Point2Pair ab3(make_pair(a3, b3));
  const Point2Pairs ab_pairs{ab1, ab2, ab3};

  std::optional<Pose2> aTb = Pose2::Align(ab_pairs);
  EXPECT(assert_equal(expected, *aTb));
}

namespace {
  /* ************************************************************************* */
  // Prototype code to align two triangles using a rigid transform
  /* ************************************************************************* */
  struct Triangle { size_t i_, j_, k_;};

  std::optional<Pose2> align2(const Point2Vector& as, const Point2Vector& bs,
    const pair<Triangle, Triangle>& trianglePair) {
      const Triangle& t1 = trianglePair.first, t2 = trianglePair.second;
      Point2Pairs ab_pairs = {{as[t1.i_], bs[t2.i_]},
                              {as[t1.j_], bs[t2.j_]},
                              {as[t1.k_], bs[t2.k_]}};
      return Pose2::Align(ab_pairs);
  }
}

TEST(Pose2, align_4) {
  using namespace align_3;

  Point2Vector as{a1, a2, a3}, bs{b3, b1, b2};  // note in 3,1,2 order !

  Triangle t1; t1.i_=0; t1.j_=1; t1.k_=2;
  Triangle t2; t2.i_=1; t2.j_=2; t2.k_=0;

  std::optional<Pose2> actual = align2(as, bs, {t1, t2});
  EXPECT(assert_equal(expected, *actual));
}

//******************************************************************************
namespace {
Pose2 id;
Pose2 T1(M_PI / 4.0, Point2(sqrt(0.5), sqrt(0.5)));
Pose2 T2(M_PI / 2.0, Point2(0.0, 2.0));
}  // namespace

//******************************************************************************
TEST(Pose2, Invariants) {
  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, T1));
  EXPECT(check_group_invariants(T2, id));
  EXPECT(check_group_invariants(T2, T1));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, T1));
  EXPECT(check_manifold_invariants(T2, id));
  EXPECT(check_manifold_invariants(T2, T1));
}

//******************************************************************************
TEST(Pose2, LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, T2);
  CHECK_LIE_GROUP_DERIVATIVES(T2, id);
  CHECK_LIE_GROUP_DERIVATIVES(T2, T1);
}

//******************************************************************************
TEST(Pose2, ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(id, id);
  CHECK_CHART_DERIVATIVES(id, T2);
  CHECK_CHART_DERIVATIVES(T2, id);
  CHECK_CHART_DERIVATIVES(T2, T1);
}

//******************************************************************************
#include "testPoseAdjointMap.h"

TEST(Pose2 , TransformCovariance3) {
  // Use simple covariance matrices and transforms to create tests that can be
  // validated with simple computations.
  using namespace test_pose_adjoint_map;

  // rotate
  {
    auto cov = FullCovarianceFromSigmas<Pose2>({0.1, 0.3, 0.7});
    auto transformed = TransformCovariance<Pose2>{{0., 0., 90 * degree}}(cov);
    // interchange x and y axes
    EXPECT(assert_equal(
      Vector3{cov(1, 1), cov(0, 0), cov(2, 2)},
      Vector3{transformed.diagonal()}));
    EXPECT(assert_equal(
      Vector3{-cov(1, 0), -cov(2, 1), cov(2, 0)},
      Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
  }

  // translate along x with uncertainty in x
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose2>(0, 0.1);
    auto transformed = TransformCovariance<Pose2>{{20., 0., 0.}}(cov);
    // No change
    EXPECT(assert_equal(cov, transformed));
  }

  // translate along x with uncertainty in y
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose2>(1, 0.1);
    auto transformed = TransformCovariance<Pose2>{{20., 0., 0.}}(cov);
    // No change
    EXPECT(assert_equal(cov, transformed));
  }

  // translate along x with uncertainty in theta
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose2>(2, 0.1);
    auto transformed = TransformCovariance<Pose2>{{20., 0., 0.}}(cov);
    EXPECT(assert_equal(
      Vector3{0., 0.1 * 0.1 * 20. * 20., 0.1 * 0.1},
      Vector3{transformed.diagonal()}));
    EXPECT(assert_equal(
      Vector3{0., 0., -0.1 * 0.1 * 20.},
      Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
  }

  // rotate and translate along x with uncertainty in x
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose2>(0, 0.1);
    auto transformed = TransformCovariance<Pose2>{{20., 0., 90 * degree}}(cov);
    // interchange x and y axes
    EXPECT(assert_equal(
      Vector3{cov(1, 1), cov(0, 0), cov(2, 2)},
      Vector3{transformed.diagonal()}));
    EXPECT(assert_equal(
      Vector3{Z_3x1},
      Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
  }

  // rotate and translate along x with uncertainty in theta
  {
    auto cov = SingleVariableCovarianceFromSigma<Pose2>(2, 0.1);
    auto transformed = TransformCovariance<Pose2>{{20., 0., 90 * degree}}(cov);
    EXPECT(assert_equal(
      Vector3{0., 0.1 * 0.1 * 20. * 20., 0.1 * 0.1},
      Vector3{transformed.diagonal()}));
    EXPECT(assert_equal(
      Vector3{0., 0., -0.1 * 0.1 * 20.},
      Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
  }
}

/* ************************************************************************* */
TEST(Pose2, Print) {
  Pose2 pose(Rot2::Identity(), Point2(1, 2));

  // Generate the expected output
  string s = "Planar Pose";
  string expected_stdout = "(1, 2, 0)";
  string expected1 = expected_stdout + "\n";
  string expected2 = s + " " + expected1;

  EXPECT(assert_stdout_equal(expected_stdout, pose));

  EXPECT(assert_print_equal(expected1, pose));
  EXPECT(assert_print_equal(expected2, pose, s));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

