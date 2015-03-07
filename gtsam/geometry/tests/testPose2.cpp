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

#include <cmath>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

using namespace gtsam;
using namespace std;

// #define SLOW_BUT_CORRECT_EXPMAP

GTSAM_CONCEPT_TESTABLE_INST(Pose2)
GTSAM_CONCEPT_LIE_INST(Pose2)

/* ************************************************************************* */
TEST(Pose2, constructors) {
  Point2 p;
  Pose2 pose(0,p);
  Pose2 origin;
  assert_equal(pose,origin);
	Pose2 t(M_PI/2.0+0.018, Point2(1.015, 2.01));
	EXPECT(assert_equal(t,Pose2(t.matrix())));
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
  Pose2 actual = pose.retract(Vector_(3, 0.01, -0.015, 0.99));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
  Pose2 expected(1.00811, 2.01528, 2.5608);
  Pose2 actual = expmap_default<Pose2>(pose, Vector_(3, 0.01, -0.015, 0.99));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap2) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
  Pose2 expected(1.00811, 2.01528, 2.5608);
  Pose2 actual = expmap_default<Pose2>(pose, Vector_(3, 0.01, -0.015, 0.99));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap3) {
  // do an actual series exponential map
	// see e.g. http://www.cis.upenn.edu/~cis610/cis610lie1.ps
  Matrix A = Matrix_(3,3,
  		0.0, -0.99,  0.01,
  		0.99,  0.0, -0.015,
  		0.0,   0.0,  0.0);
  Matrix A2 = A*A/2.0, A3 = A2*A/3.0, A4=A3*A/4.0;
  Matrix expected = eye(3) + A + A2 + A3 + A4;

  Vector v = Vector_(3, 0.01, -0.015, 0.99);
  Pose2 pose = Pose2::Expmap(v);
  Pose2 pose2(v);
  EXPECT(assert_equal(pose, pose2));
  Matrix actual = pose.matrix();
  //EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose2, expmap0) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
//#ifdef SLOW_BUT_CORRECT_EXPMAP
  Pose2 expected(1.01491, 2.01013, 1.5888);
//#else
//  Pose2 expected(M_PI/2.0+0.018, Point2(1.015, 2.01));
//#endif
  Pose2 actual = pose * (Pose2::Expmap(Vector_(3, 0.01, -0.015, 0.018)));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap0_full) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
  Pose2 expected(1.01491, 2.01013, 1.5888);
  Pose2 actual = pose * Pose2::Expmap(Vector_(3, 0.01, -0.015, 0.018));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap0_full2) {
  Pose2 pose(M_PI/2.0, Point2(1, 2));
  Pose2 expected(1.01491, 2.01013, 1.5888);
  Pose2 actual = pose * Pose2::Expmap(Vector_(3, 0.01, -0.015, 0.018));
  EXPECT(assert_equal(expected, actual, 1e-5));
}

#ifdef SLOW_BUT_CORRECT_EXPMAP
/* ************************************************************************* */
// test case for screw motion in the plane
namespace screw {
  double w=0.3;
	Vector xi = Vector_(3, 0.0, w, w);
	Rot2 expectedR = Rot2::fromAngle(w);
	Point2 expectedT(-0.0446635, 0.29552);
	Pose2 expected(expectedR, expectedT);
}

TEST(Pose3, expmap_c)
{
  EXPECT(assert_equal(screw::expected, expm<Pose2>(screw::xi),1e-6));
  EXPECT(assert_equal(screw::expected, Pose2::Expmap(screw::xi),1e-6));
  EXPECT(assert_equal(screw::xi, Pose2::Logmap(screw::expected),1e-6));
}
#endif

/* ************************************************************************* */
TEST(Pose2, expmap_c_full)
{
  double w=0.3;
	Vector xi = Vector_(3, 0.0, w, w);
	Rot2 expectedR = Rot2::fromAngle(w);
	Point2 expectedT(-0.0446635, 0.29552);
	Pose2 expected(expectedR, expectedT);
  EXPECT(assert_equal(expected, expm<Pose2>(xi),1e-6));
  EXPECT(assert_equal(expected, Pose2::Expmap(xi),1e-6));
  EXPECT(assert_equal(xi, Pose2::Logmap(expected),1e-6));
}

/* ************************************************************************* */
TEST(Pose2, logmap) {
  Pose2 pose0(M_PI/2.0, Point2(1, 2));
  Pose2 pose(M_PI/2.0+0.018, Point2(1.015, 2.01));
#ifdef SLOW_BUT_CORRECT_EXPMAP
  Vector expected = Vector_(3, 0.00986473, -0.0150896, 0.018);
#else
  Vector expected = Vector_(3, 0.01, -0.015, 0.018);
#endif
  Vector actual = pose0.localCoordinates(pose);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, logmap_full) {
  Pose2 pose0(M_PI/2.0, Point2(1, 2));
  Pose2 pose(M_PI/2.0+0.018, Point2(1.015, 2.01));
  Vector expected = Vector_(3, 0.00986473, -0.0150896, 0.018);
  Vector actual = logmap_default<Pose2>(pose0, pose);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
static Point2 transform_to_proxy(const Pose2& pose, const Point2& point) {
	return pose.transform_to(point);
}

TEST( Pose2, transform_to )
{
  Pose2 pose(M_PI/2.0, Point2(1,2)); // robot at (1,2) looking towards y
  Point2 point(-1,4);    // landmark at (-1,4)

  // expected
  Point2 expected(2,2);
  Matrix expectedH1 = Matrix_(2,3, -1.0, 0.0, 2.0,  0.0, -1.0, -2.0);
  Matrix expectedH2 = Matrix_(2,2, 0.0, 1.0,  -1.0, 0.0);

  // actual
  Matrix actualH1, actualH2;
  Point2 actual = pose.transform_to(point, actualH1, actualH2);
  EXPECT(assert_equal(expected,actual));

  EXPECT(assert_equal(expectedH1,actualH1));
  Matrix numericalH1 = numericalDerivative21(transform_to_proxy, pose, point);
  EXPECT(assert_equal(numericalH1,actualH1));

  EXPECT(assert_equal(expectedH2,actualH2));
  Matrix numericalH2 = numericalDerivative22(transform_to_proxy, pose, point);
  EXPECT(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
static Point2 transform_from_proxy(const Pose2& pose, const Point2& point) {
	return pose.transform_from(point);
}

TEST (Pose2, transform_from)
{
	Pose2 pose(1., 0., M_PI/2.0);
	Point2 pt(2., 1.);
	Matrix H1, H2;
	Point2 actual = pose.transform_from(pt, H1, H2);

	Point2 expected(0., 2.);
	EXPECT(assert_equal(expected, actual));

	Matrix H1_expected = Matrix_(2, 3, 0., -1., -2., 1., 0., -1.);
	Matrix H2_expected = Matrix_(2, 2, 0., -1., 1., 0.);

	Matrix numericalH1 = numericalDerivative21(transform_from_proxy, pose, pt);
	EXPECT(assert_equal(H1_expected, H1));
	EXPECT(assert_equal(H1_expected, numericalH1));

	Matrix numericalH2 = numericalDerivative22(transform_from_proxy, pose, pt);
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

  Matrix expectedH1 = Matrix_(3,3,
  		0.0, 1.0, 0.0,
  	   -1.0, 0.0, 2.0,
  		0.0, 0.0, 1.0
  );
  Matrix expectedH2 = eye(3);
  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(testing::compose, pose1, pose2);
  EXPECT(assert_equal(expectedH1,actualDcompose1));
  EXPECT(assert_equal(numericalH1,actualDcompose1));
  EXPECT(assert_equal(expectedH2,actualDcompose2));
  EXPECT(assert_equal(numericalH2,actualDcompose2));

  Point2 point(sqrt(0.5), 3.0*sqrt(0.5));
  Point2 expected_point(-1.0, -1.0);
  Point2 actual_point1 = (pose1 * pose2).transform_to(point);
  Point2 actual_point2 = pose2.transform_to(pose1.transform_to(point));
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
	Point2 origin, t(1,2);
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

/* ************************************************************************* */
Vector homogeneous(const Point2& p) {
	return Vector_(3, p.x(), p.y(), 1.0);
}

/* ************************************************************************* */
Matrix matrix(const Pose2& gTl) {
	Matrix gRl = gTl.r().matrix();
	Point2 gt = gTl.t();
	return Matrix_(3, 3,
			gRl(0, 0), gRl(0, 1), gt.x(),
			gRl(1, 0), gRl(1, 1), gt.y(),
			      0.0,       0.0,   1.0);
}

/* ************************************************************************* */
TEST( Pose2, matrix )
{
	Point2 origin, t(1,2);
	Pose2 gTl(M_PI/2.0, t); // robot at (1,2) looking towards y
  Matrix gMl = matrix(gTl);
  EXPECT(assert_equal(Matrix_(3,3,
  		0.0, -1.0, 1.0,
  		1.0,  0.0, 2.0,
  		0.0,  0.0, 1.0),
  		gMl));
  Rot2 gR1 = gTl.r();
  EXPECT(assert_equal(homogeneous(t),gMl*homogeneous(origin)));
  Point2 x_axis(1,0), y_axis(0,1);
  EXPECT(assert_equal(Matrix_(2,2,
  		0.0, -1.0,
  		1.0,  0.0),
  		gR1.matrix()));
  EXPECT(assert_equal(Point2(0,1),gR1*x_axis));
  EXPECT(assert_equal(Point2(-1,0),gR1*y_axis));
  EXPECT(assert_equal(homogeneous(Point2(1+0,2+1)),gMl*homogeneous(x_axis)));
  EXPECT(assert_equal(homogeneous(Point2(1-1,2+0)),gMl*homogeneous(y_axis)));

  // check inverse pose
  Matrix lMg = matrix(gTl.inverse());
  EXPECT(assert_equal(Matrix_(3,3,
  		0.0,  1.0,-2.0,
  	 -1.0,  0.0, 1.0,
  		0.0,  0.0, 1.0),
  		lMg));
}

/* ************************************************************************* */
TEST( Pose2, compose_matrix )
{
  Pose2 gT1(M_PI/2.0, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 _1T2(M_PI, Point2(-1,4));  // local robot at (-1,4) loooking at negative x
  Matrix gM1(matrix(gT1)),_1M2(matrix(_1T2));
  EXPECT(assert_equal(gM1*_1M2,matrix(gT1.compose(_1T2)))); // RIGHT DOES NOT
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
  Pose2 gT2(M_PI, Point2(-1,4));  // robot at (-1,4) loooking at negative x

  Matrix actualH1,actualH2;
  Pose2 expected(M_PI/2.0, Point2(2,2));
  Pose2 actual1 = gT1.between(gT2);
  Pose2 actual2 = gT1.between(gT2,actualH1,actualH2);
  EXPECT(assert_equal(expected,actual1));
  EXPECT(assert_equal(expected,actual2));

  Matrix expectedH1 = Matrix_(3,3,
      0.0,-1.0,-2.0,
      1.0, 0.0,-2.0,
      0.0, 0.0,-1.0
  );
  Matrix numericalH1 = numericalDerivative21<Pose2,Pose2,Pose2>(testing::between, gT1, gT2);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(numericalH1,actualH1));
	// Assert H1 = -AdjointMap(between(p2,p1)) as in doc/math.lyx
  EXPECT(assert_equal(-gT2.between(gT1).adjointMap(),actualH1));

  Matrix expectedH2 = Matrix_(3,3,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0
  );
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
TEST( Pose2, round_trip )
{
	Pose2 p1(1.23, 2.30, 0.2);
	Pose2 odo(0.53, 0.39, 0.15);
	Pose2 p2 = p1.compose(odo);
	EXPECT(assert_equal(odo, p1.between(p2)));
}

/* ************************************************************************* */
TEST(Pose2, members)
{
  Pose2 pose;
  EXPECT(pose.dim() == 3);
}

/* ************************************************************************* */
// some shared test values
Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI/4.0);
Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

/* ************************************************************************* */
Rot2 bearing_proxy(const Pose2& pose, const Point2& pt) {
	return pose.bearing(pt);
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
	EXPECT(assert_equal(expectedH1,actualH1));

	// establish bearing is indeed 45 degrees even if rotated
	Rot2 actual34 = x3.bearing(l4, actualH1, actualH2);
	EXPECT(assert_equal(Rot2::fromAngle(M_PI/4.0),actual34));

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(bearing_proxy, x3, l4);
	expectedH2 = numericalDerivative22(bearing_proxy, x3, l4);
	EXPECT(assert_equal(expectedH1,actualH1));
	EXPECT(assert_equal(expectedH1,actualH1));
}

/* ************************************************************************* */
Rot2 bearing_pose_proxy(const Pose2& pose, const Pose2& pt) {
	return pose.bearing(pt);
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
LieVector range_proxy(const Pose2& pose, const Point2& point) {
	return LieVector(pose.range(point));
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
LieVector range_pose_proxy(const Pose2& pose, const Pose2& point) {
	return LieVector(pose.range(point));
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
	Pose2 expected(Rot2::fromAngle(0), Point2(10,10));

	vector<Point2Pair> correspondences;
	Point2Pair pq1(make_pair(Point2(0,0), Point2(10,10)));
	Point2Pair pq2(make_pair(Point2(20,10), Point2(30,20)));
	correspondences += pq1, pq2;

  boost::optional<Pose2> actual = align(correspondences);
  EXPECT(assert_equal(expected, *actual));
}

TEST(Pose2, align_2) {
	Point2 t(20,10);
	Rot2 R = Rot2::fromAngle(M_PI/2.0);
	Pose2 expected(R, t);

	vector<Point2Pair> correspondences;
	Point2 p1(0,0), p2(10,0);
	Point2 q1 = expected.transform_from(p1), q2 = expected.transform_from(p2);
  EXPECT(assert_equal(Point2(20,10),q1));
  EXPECT(assert_equal(Point2(20,20),q2));
	Point2Pair pq1(make_pair(p1, q1));
	Point2Pair pq2(make_pair(p2, q2));
	correspondences += pq1, pq2;

  boost::optional<Pose2> actual = align(correspondences);
  EXPECT(assert_equal(expected, *actual));
}

namespace align_3 {
	Point2 t(10,10);
	Pose2 expected(Rot2::fromAngle(2*M_PI/3), t);
	Point2 p1(0,0), p2(10,0), p3(10,10);
	Point2 q1 = expected.transform_from(p1), q2 = expected.transform_from(p2), q3 = expected.transform_from(p3);
}

TEST(Pose2, align_3) {
	using namespace align_3;

	vector<Point2Pair> correspondences;
	Point2Pair pq1(make_pair(p1, q1));
	Point2Pair pq2(make_pair(p2, q2));
	Point2Pair pq3(make_pair(p3, q3));
	correspondences += pq1, pq2, pq3;

  boost::optional<Pose2> actual = align(correspondences);
  EXPECT(assert_equal(expected, *actual));
}

/* ************************************************************************* */
// Prototype code to align two triangles using a rigid transform
/* ************************************************************************* */
struct Triangle { size_t i_,j_,k_;};

boost::optional<Pose2> align(const vector<Point2>& ps, const vector<Point2>& qs,
		const pair<Triangle, Triangle>& trianglePair) {
	const Triangle& t1 = trianglePair.first, t2 = trianglePair.second;
	vector<Point2Pair> correspondences;
	correspondences += make_pair(ps[t1.i_],qs[t2.i_]), make_pair(ps[t1.j_],qs[t2.j_]), make_pair(ps[t1.k_],qs[t2.k_]);
	return align(correspondences);
}

TEST(Pose2, align_4) {
	using namespace align_3;

	vector<Point2> ps,qs;
	ps += p1, p2, p3;
	qs += q3, q1, q2; // note in 3,1,2 order !

	Triangle t1; t1.i_=0; t1.j_=1; t1.k_=2;
	Triangle t2; t2.i_=1; t2.j_=2; t2.k_=0;

  boost::optional<Pose2> actual = align(ps, qs, make_pair(t1,t2));
  EXPECT(assert_equal(expected, *actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

