/**
 * @file   testPose2.cpp
 * @brief  Unit tests for Pose2 class
 */

#include <math.h>
#include <gtsam/CppUnitLite/TestHarness.h>
#include <iostream>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

using namespace gtsam;
using namespace std;

// #define SLOW_BUT_CORRECT_EXPMAP

/* ************************************************************************* */
TEST(Pose2, constructors) {
  //cout << "constructors" << endl;
  Point2 p;
  Pose2 pose(0,p);
  Pose2 origin;
  assert_equal(pose,origin);
	Pose2 t(M_PI_2+0.018, Point2(1.015, 2.01));
	CHECK(assert_equal(t,Pose2(t.matrix())));
}

/* ************************************************************************* */
TEST(Pose2, manifold) {
  //cout << "manifold" << endl;
	Pose2 t1(M_PI_2, Point2(1, 2));
	Pose2 t2(M_PI_2+0.018, Point2(1.015, 2.01));
	Pose2 origin;
	Vector d12 = logmap(t1,t2);
	CHECK(assert_equal(t2, expmap(t1,d12)));
	CHECK(assert_equal(t2, t1*expmap(origin,d12)));
	Vector d21 = logmap(t2,t1);
	CHECK(assert_equal(t1, expmap(t2,d21)));
	CHECK(assert_equal(t1, t2*expmap(origin,d21)));
}

/* ************************************************************************* */
TEST(Pose2, expmap) {
  //cout << "expmap" << endl;
  Pose2 pose(M_PI_2, Point2(1, 2));
#ifdef SLOW_BUT_CORRECT_EXPMAP
  Pose2 expected(1.00811, 2.01528, 2.5608);
#else
  Pose2 expected(M_PI_2+0.99, Point2(1.015, 2.01));
#endif
  Pose2 actual = expmap(pose, Vector_(3, 0.01, -0.015, 0.99));
  CHECK(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(Pose2, expmap2) {
  // do an actual series exponential map
	// see e.g. http://www.cis.upenn.edu/~cis610/cis610lie1.ps
  Matrix A = Matrix_(3,3,
  		0.0, -0.99,  0.01,
  		0.99,  0.0, -0.015,
  		0.0,   0.0,  0.0);
  Matrix A2 = A*A/2.0, A3 = A2*A/3.0, A4=A3*A/4.0;
  Matrix expected = eye(3) + A + A2 + A3 + A4;

  Pose2 pose = expmap<Pose2>(Vector_(3, 0.01, -0.015, 0.99));
  Matrix actual = pose.matrix();
  //CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose2, expmap0) {
  //cout << "expmap0" << endl;
  Pose2 pose(M_PI_2, Point2(1, 2));
#ifdef SLOW_BUT_CORRECT_EXPMAP
  Pose2 expected(1.01491, 2.01013, 1.5888);
#else
  Pose2 expected(M_PI_2+0.018, Point2(1.015, 2.01));
#endif
  Pose2 actual = pose * expmap<Pose2>(Vector_(3, 0.01, -0.015, 0.018));
  CHECK(assert_equal(expected, actual, 1e-5));
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
  CHECK(assert_equal(screw::expected, expm<Pose2>(screw::xi),1e-6));
  CHECK(assert_equal(screw::expected, expmap<Pose2>(screw::xi),1e-6));
  CHECK(assert_equal(screw::xi, logmap(screw::expected),1e-6));
}
#endif

/* ************************************************************************* */
TEST(Pose2, logmap) {
  //cout << "logmap" << endl;
  Pose2 pose0(M_PI_2, Point2(1, 2));
  Pose2 pose(M_PI_2+0.018, Point2(1.015, 2.01));
#ifdef SLOW_BUT_CORRECT_EXPMAP
  Vector expected = Vector_(3, 0.00986473, -0.0150896, 0.018);
#else
  Vector expected = Vector_(3, 0.01, -0.015, 0.018);
#endif
  Vector actual = logmap(pose0,pose);
  CHECK(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
Point2 transform_to_proxy(const Pose2& pose, const Point2& point) {
	return pose.transform_to(point);
}

TEST( Pose2, transform_to )
{
  Pose2 pose(M_PI_2, Point2(1,2)); // robot at (1,2) looking towards y
  Point2 point(-1,4);    // landmark at (-1,4)

  // expected
  Point2 expected(2,2);
  Matrix expectedH1 = Matrix_(2,3, -1.0, 0.0, 2.0,  0.0, -1.0, -2.0);
  Matrix expectedH2 = Matrix_(2,2, 0.0, 1.0,  -1.0, 0.0);

  // actual
  Matrix actualH1, actualH2;
  Point2 actual = pose.transform_to(point, actualH1, actualH2);
  CHECK(assert_equal(expected,actual));

  CHECK(assert_equal(expectedH1,actualH1));
  Matrix numericalH1 = numericalDerivative21(transform_to_proxy, pose, point, 1e-5);
  CHECK(assert_equal(numericalH1,actualH1));

  CHECK(assert_equal(expectedH2,actualH2));
  Matrix numericalH2 = numericalDerivative22(transform_to_proxy, pose, point, 1e-5);
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
Point2 transform_from_proxy(const Pose2& pose, const Point2& point) {
	return pose.transform_from(point);
}

TEST (Pose2, transform_from)
{
	Pose2 pose(1., 0., M_PI_2);
	Point2 pt(2., 1.);
	Matrix H1, H2;
	Point2 actual = pose.transform_from(pt, H1, H2);

	Point2 expected(0., 2.);
	CHECK(assert_equal(expected, actual));

	Matrix H1_expected = Matrix_(2, 3, 0., -1., -2., 1., 0., -1.);
	Matrix H2_expected = Matrix_(2, 2, 0., -1., 1., 0.);

	Matrix numericalH1 = numericalDerivative21(transform_from_proxy, pose, pt, 1e-5);
	CHECK(assert_equal(H1_expected, H1));
	CHECK(assert_equal(H1_expected, numericalH1));

	Matrix numericalH2 = numericalDerivative22(transform_from_proxy, pose, pt, 1e-5);
	CHECK(assert_equal(H2_expected, H2));
	CHECK(assert_equal(H2_expected, numericalH2));
}

/* ************************************************************************* */
TEST(Pose2, compose_a)
{
  //cout << "compose_a" << endl;
  Pose2 pose1(M_PI/4.0, Point2(sqrt(0.5), sqrt(0.5)));
  Pose2 pose2(M_PI/2.0, Point2(0.0, 2.0));

  Matrix actualDcompose1;
  Matrix actualDcompose2;
  Pose2 actual = compose(pose1, pose2, actualDcompose1, actualDcompose2);

  Pose2 expected(3.0*M_PI/4.0, Point2(-sqrt(0.5), 3.0*sqrt(0.5)));
  CHECK(assert_equal(expected, actual));

  Matrix expectedH1 = Matrix_(3,3,
  		0.0, 1.0, 0.0,
  	 -1.0, 0.0, 2.0,
  		0.0, 0.0, 1.0
  );
  Matrix expectedH2 = eye(3);
  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(compose, pose1, pose2, 1e-5);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(compose, pose1, pose2, 1e-5);
  CHECK(assert_equal(expectedH1,actualDcompose1));
  CHECK(assert_equal(numericalH1,actualDcompose1));
  CHECK(assert_equal(expectedH2,actualDcompose2));
  CHECK(assert_equal(numericalH2,actualDcompose2));

  Point2 point(sqrt(0.5), 3.0*sqrt(0.5));
  Point2 expected_point(-1.0, -1.0);
  Point2 actual_point1 = (pose1 * pose2).transform_to(point);
  Point2 actual_point2 = pose2.transform_to(pose1.transform_to(point));
  CHECK(assert_equal(expected_point, actual_point1));
  CHECK(assert_equal(expected_point, actual_point2));
}

/* ************************************************************************* */
TEST(Pose2, compose_b)
{
  //cout << "compose_b" << endl;
  Pose2 pose1(Rot2::fromAngle(M_PI/10.0), Point2(.75, .5));
  Pose2 pose2(Rot2::fromAngle(M_PI/4.0-M_PI/10.0), Point2(0.701289620636, 1.34933052585));

  Pose2 pose_expected(Rot2::fromAngle(M_PI/4.0), Point2(1.0, 2.0));

  Pose2 pose_actual_op = pose1 * pose2;
  Matrix actualDcompose1, actualDcompose2;
  Pose2 pose_actual_fcn = compose(pose1, pose2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(compose, pose1, pose2, 1e-5);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(compose, pose1, pose2, 1e-5);
  CHECK(assert_equal(numericalH1,actualDcompose1,1e-5));
  CHECK(assert_equal(numericalH2,actualDcompose2));

  CHECK(assert_equal(pose_expected, pose_actual_op));
  CHECK(assert_equal(pose_expected, pose_actual_fcn));
}

/* ************************************************************************* */
TEST(Pose2, compose_c)
{
  //cout << "compose_c" << endl;
  Pose2 pose1(Rot2::fromAngle(M_PI/4.0), Point2(1.0, 1.0));
  Pose2 pose2(Rot2::fromAngle(M_PI/4.0), Point2(sqrt(.5), sqrt(.5)));

  Pose2 pose_expected(Rot2::fromAngle(M_PI/2.0), Point2(1.0, 2.0));

  Pose2 pose_actual_op = pose1 * pose2;
  Matrix actualDcompose1, actualDcompose2;
  Pose2 pose_actual_fcn = compose(pose1,pose2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21<Pose2, Pose2, Pose2>(compose, pose1, pose2, 1e-5);
  Matrix numericalH2 = numericalDerivative22<Pose2, Pose2, Pose2>(compose, pose1, pose2, 1e-5);
  CHECK(assert_equal(numericalH1,actualDcompose1,1e-5));
  CHECK(assert_equal(numericalH2,actualDcompose2));

  CHECK(assert_equal(pose_expected, pose_actual_op));
  CHECK(assert_equal(pose_expected, pose_actual_fcn));
}

/* ************************************************************************* */
TEST(Pose2, inverse )
{
	Point2 origin, t(1,2);
	Pose2 gTl(M_PI_2, t); // robot at (1,2) looking towards y

	Pose2 identity, lTg = inverse(gTl);
	CHECK(assert_equal(identity,compose(lTg,gTl)));
	CHECK(assert_equal(identity,compose(gTl,lTg)));

	Point2 l(4,5), g(-4,6);
	CHECK(assert_equal(g,gTl*l));
	CHECK(assert_equal(l,lTg*g));

	// Check derivative
  Matrix numericalH = numericalDerivative11<Pose2,Pose2>(inverse, lTg, 1e-5);
  Matrix actualDinverse;
  inverse(lTg, actualDinverse);
  CHECK(assert_equal(numericalH,actualDinverse));
}

/* ************************************************************************* */
Vector homogeneous(const Point2& p) {
	return Vector_(3, p.x(), p.y(), 1.0);
}

Matrix matrix(const Pose2& gTl) {
	Matrix gRl = gTl.r().matrix();
	Point2 gt = gTl.t();
	return Matrix_(3, 3,
			gRl(0, 0), gRl(0, 1), gt.x(),
			gRl(1, 0), gRl(1, 1), gt.y(),
			      0.0,       0.0,   1.0);
}

TEST( Pose2, matrix )
{
	Point2 origin, t(1,2);
	Pose2 gTl(M_PI_2, t); // robot at (1,2) looking towards y
  Matrix gMl = matrix(gTl);
  CHECK(assert_equal(Matrix_(3,3,
  		0.0, -1.0, 1.0,
  		1.0,  0.0, 2.0,
  		0.0,  0.0, 1.0),
  		gMl));
  Rot2 gR1 = gTl.r();
  CHECK(assert_equal(homogeneous(t),gMl*homogeneous(origin)));
  Point2 x_axis(1,0), y_axis(0,1);
  CHECK(assert_equal(Matrix_(2,2,
  		0.0, -1.0,
  		1.0,  0.0),
  		gR1.matrix()));
  CHECK(assert_equal(Point2(0,1),gR1*x_axis));
  CHECK(assert_equal(Point2(-1,0),gR1*y_axis));
  CHECK(assert_equal(homogeneous(Point2(1+0,2+1)),gMl*homogeneous(x_axis)));
  CHECK(assert_equal(homogeneous(Point2(1-1,2+0)),gMl*homogeneous(y_axis)));

  // check inverse pose
  Matrix lMg = matrix(inverse(gTl));
  CHECK(assert_equal(Matrix_(3,3,
  		0.0,  1.0,-2.0,
  	 -1.0,  0.0, 1.0,
  		0.0,  0.0, 1.0),
  		lMg));
}

/* ************************************************************************* */
TEST( Pose2, compose_matrix )
{
  Pose2 gT1(M_PI_2, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 _1T2(M_PI, Point2(-1,4));  // local robot at (-1,4) loooking at negative x
  Matrix gM1(matrix(gT1)),_1M2(matrix(_1T2));
  CHECK(assert_equal(gM1*_1M2,matrix(compose(gT1,_1T2)))); // RIGHT DOES NOT
}

/* ************************************************************************* */
TEST( Pose2, between )
{
  // <
  //
	//       ^
	//
	// *--0--*--*
  Pose2 gT1(M_PI_2, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 gT2(M_PI, Point2(-1,4));  // robot at (-1,4) loooking at negative x

  Matrix actualH1,actualH2;
  Pose2 expected(M_PI_2, Point2(2,2));
  Pose2 actual1 = between(gT1,gT2);
  Pose2 actual2 = between(gT1,gT2,actualH1,actualH2);
  CHECK(assert_equal(expected,actual1));
  CHECK(assert_equal(expected,actual2));

  Matrix expectedH1 = Matrix_(3,3,
      0.0,-1.0,-2.0,
      1.0, 0.0,-2.0,
      0.0, 0.0,-1.0
  );
  Matrix numericalH1 = numericalDerivative21(between<Pose2>, gT1, gT2, 1e-5);
  CHECK(assert_equal(expectedH1,actualH1));
  CHECK(assert_equal(numericalH1,actualH1));
	// Assert H1 = -AdjointMap(between(p2,p1)) as in doc/math.lyx
  CHECK(assert_equal(-between(gT2,gT1).AdjointMap(),actualH1));

  Matrix expectedH2 = Matrix_(3,3,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0
  );
  Matrix numericalH2 = numericalDerivative22(between<Pose2>, gT1, gT2, 1e-5);
  CHECK(assert_equal(expectedH2,actualH2));
  CHECK(assert_equal(numericalH2,actualH2));

}

/* ************************************************************************* */
// reverse situation for extra test
TEST( Pose2, between2 )
{
  Pose2 p2(M_PI_2, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 p1(M_PI, Point2(-1,4));  // robot at (-1,4) loooking at negative x

  Matrix actualH1,actualH2;
  between(p1,p2,actualH1,actualH2);
  Matrix numericalH1 = numericalDerivative21(between<Pose2>, p1, p2, 1e-5);
  CHECK(assert_equal(numericalH1,actualH1));
  Matrix numericalH2 = numericalDerivative22(between<Pose2>, p1, p2, 1e-5);
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST( Pose2, round_trip )
{
	Pose2 p1(1.23, 2.30, 0.2);
	Pose2 odo(0.53, 0.39, 0.15);
	Pose2 p2 = compose(p1, odo);
	CHECK(assert_equal(odo, between(p1, p2)));
}

/* ************************************************************************* */
TEST(Pose2, members)
{
  Pose2 pose;
  CHECK(pose.dim() == 3);
}

/* ************************************************************************* */
// some shared test values
Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

/* ************************************************************************* */
Rot2 bearing_proxy(const Pose2& pose, const Point2& pt) {
	return pose.bearing(pt);
}

TEST( Pose2, bearing )
{
	Matrix expectedH1, actualH1, expectedH2, actualH2;

	// establish bearing is indeed zero
	CHECK(assert_equal(Rot2(),x1.bearing(l1)));

	// establish bearing is indeed 45 degrees
	CHECK(assert_equal(Rot2::fromAngle(M_PI_4),x1.bearing(l2)));

	// establish bearing is indeed 45 degrees even if shifted
	Rot2 actual23 = x2.bearing(l3, actualH1, actualH2);
	CHECK(assert_equal(Rot2::fromAngle(M_PI_4),actual23));

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(bearing_proxy, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	expectedH2 = numericalDerivative22(bearing_proxy, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));

	// establish bearing is indeed 45 degrees even if rotated
	Rot2 actual34 = x3.bearing(l4, actualH1, actualH2);
	CHECK(assert_equal(Rot2::fromAngle(M_PI_4),actual34));

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(bearing_proxy, x3, l4, 1e-5);
	expectedH2 = numericalDerivative22(bearing_proxy, x3, l4, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH1,actualH1));
}

/* ************************************************************************* */
LieVector range_proxy(const Pose2& pose, const Point2& point) {
	return LieVector(Vector_(1, pose.range(point)));
}
TEST( Pose2, range )
{
	Matrix expectedH1, actualH1, expectedH2, actualH2;

	// establish range is indeed zero
	DOUBLES_EQUAL(1,x1.range(l1),1e-9);

	// establish range is indeed 45 degrees
	DOUBLES_EQUAL(sqrt(2),x1.range(l2),1e-9);

	// Another pair
	double actual23 = x2.range(l3, actualH1, actualH2);
	DOUBLES_EQUAL(sqrt(2),actual23,1e-9);

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(range_proxy, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	expectedH2 = numericalDerivative22(range_proxy, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));

	// Another test
	double actual34 = x3.range(l4, actualH1, actualH2);
	DOUBLES_EQUAL(2,actual34,1e-9);

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(range_proxy, x3, l4, 1e-5);
	expectedH2 = numericalDerivative22(range_proxy, x3, l4, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH1,actualH1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

