/**
 * @file   testPose2.cpp
 * @brief  Unit tests for Pose2 class
 */

#include <math.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>
#include "numericalDerivative.h"
#include "Pose2.h"
#include "Point2.h"
#include "Rot2.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(Pose2, constructors) {
  //cout << "constructors" << endl;
  Point2 p;
  Pose2 pose(0,p);
  Pose2 origin;
  assert_equal(pose,origin);
}

/* ************************************************************************* */
TEST(Pose2, manifold) {
  //cout << "manifold" << endl;
	Pose2 t1(M_PI_2, Point2(1, 2));
	Pose2 t2(M_PI_2+0.018, Point2(1.015, 2.01));
	Pose2 origin;
	Vector d12 = logmap(t1,t2);
	CHECK(assert_equal(t2, expmap(t1,d12)));
	CHECK(assert_equal(t2, expmap(origin,d12)*t1));
	Vector d21 = logmap(t2,t1);
	CHECK(assert_equal(t1, expmap(t2,d21)));
	CHECK(assert_equal(t1, expmap(origin,d21)*t2));
}

/* ************************************************************************* */
TEST(Pose2, expmap) {
  //cout << "expmap" << endl;
  Pose2 pose(M_PI_2, Point2(1, 2));
  Pose2 expected(M_PI_2+0.018, Point2(1.015, 2.01));
  Pose2 actual = expmap(pose, Vector_(3, 0.01, -0.015, 0.018));
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose2, expmap0) {
  //cout << "expmap0" << endl;
  Pose2 pose(M_PI_2, Point2(1, 2));
  Pose2 expected(M_PI_2+0.018, Point2(1.015, 2.01));
  Pose2 actual = expmap<Pose2>(Vector_(3, 0.01, -0.015, 0.018)) * pose;
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Pose2, logmap) {
  //cout << "logmap" << endl;
  Pose2 pose0(M_PI_2, Point2(1, 2));
  Pose2 pose(M_PI_2+0.018, Point2(1.015, 2.01));
  Vector expected = Vector_(3, 0.01, -0.015, 0.018);
  Vector actual = logmap(pose0,pose);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
//TEST(Pose2, rotate) {
//  std::cout << "rotate\n";
//	double theta = 0.1, c=cos(theta),s=sin(theta);
//	Pose2 p1(1,0,0.2), p2(0,1,0.4);
//	CHECK(assert_equal(Pose2( c,s,0.3),p1.rotate(theta)));
//	CHECK(assert_equal(Pose2(-s,c,0.5),p2.rotate(theta)));
//}

/* ************************************************************************* */
TEST( Pose2, transform_to )
{
  //cout << "transform_to" << endl;
  Pose2 pose(M_PI_2, Point2(1,2)); // robot at (1,2) looking towards y
  Point2 point(-1,4);    // landmark at (-1,4)

  // expected
  Point2 expected(2,2);
//  Matrix expectedH1 = Matrix_(2,3, 0.0, -1.0, 2.0,  1.0, 0.0, -2.0);
//  Matrix expectedH2 = Matrix_(2,2, 0.0, 1.0,  -1.0, 0.0);

  // actual
  Point2 actual = transform_to(pose,point);
  Matrix actualH1 = Dtransform_to1(pose,point);
  Matrix actualH2 = Dtransform_to2(pose,point);

  CHECK(assert_equal(expected,actual));
//  CHECK(assert_equal(expectedH1,actualH1));
//  CHECK(assert_equal(expectedH2,actualH2));

  Matrix numericalH1 = numericalDerivative21(transform_to, pose, point, 1e-5);
  CHECK(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative22(transform_to, pose, point, 1e-5);
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST(Pose2, compose_a)
{
  //cout << "compose_a" << endl;
  Pose2 pose1(M_PI/4.0, Point2(sqrt(0.5), sqrt(0.5)));
  Pose2 pose2(M_PI/2.0, Point2(0.0, 2.0));

  Pose2 expected(3.0*M_PI/4.0, Point2(-sqrt(0.5), 3.0*sqrt(0.5)));
  Pose2 actual = pose2 * pose1;
  CHECK(assert_equal(expected, actual));

  Point2 point(sqrt(0.5), 3.0*sqrt(0.5));
  Point2 expected_point(-1.0, -1.0);
  Point2 actual_point1 = transform_to(pose2 * pose1, point);
  Point2 actual_point2 = transform_to(pose2, transform_to(pose1, point));
  CHECK(assert_equal(expected_point, actual_point1));
  CHECK(assert_equal(expected_point, actual_point2));
}

/* ************************************************************************* */
TEST(Pose2, compose_b)
{
  //cout << "compose_b" << endl;
  Pose2 pose1(Rot2(M_PI/10.0), Point2(.75, .5));
  Pose2 pose2(Rot2(M_PI/4.0-M_PI/10.0), Point2(0.701289620636, 1.34933052585));

  Pose2 pose_expected(Rot2(M_PI/4.0), Point2(1.0, 2.0));

  Pose2 pose_actual_op = pose2 * pose1;
  Pose2 pose_actual_fcn = compose(pose2,pose1);

  CHECK(assert_equal(pose_expected, pose_actual_op));
  CHECK(assert_equal(pose_expected, pose_actual_fcn));
}

/* ************************************************************************* */
TEST(Pose2, compose_c)
{
  //cout << "compose_c" << endl;
  Pose2 pose1(Rot2(M_PI/4.0), Point2(1.0, 1.0));
  Pose2 pose2(Rot2(M_PI/4.0), Point2(sqrt(.5), sqrt(.5)));

  Pose2 pose_expected(Rot2(M_PI/2.0), Point2(1.0, 2.0));

  Pose2 pose_actual_op = pose2 * pose1;
  Pose2 pose_actual_fcn = compose(pose2,pose1);

  CHECK(assert_equal(pose_expected, pose_actual_op));
  CHECK(assert_equal(pose_expected, pose_actual_fcn));
}


/* ************************************************************************* */
TEST( Pose2, between )
{
  //cout << "between" << endl;
  Pose2 p1(M_PI_2, Point2(1,2)); // robot at (1,2) looking towards y
  Pose2 p2(M_PI, Point2(-1,4));  // robot at (-1,4) loooking at negative x

  Pose2 expected(M_PI_2, Point2(2,2));
  Pose2 actual = between(p1,p2);
  CHECK(assert_equal(expected,actual));

  Matrix expectedH1 = Matrix_(3,3,
      0.0,-1.0,-2.0,
      1.0, 0.0,-2.0,
      0.0, 0.0,-1.0
  );
  Matrix numericalH1 = numericalDerivative21(between<Pose2>, p1, p2, 1e-5);
  Matrix actualH1 = Dbetween1(p1,p2);
  CHECK(assert_equal(expectedH1,actualH1));
  CHECK(assert_equal(numericalH1,actualH1));

  Matrix expectedH2 = Matrix_(3,3,
       1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0
  );
  Matrix numericalH2 = numericalDerivative22(between<Pose2>, p1, p2, 1e-5);
  Matrix actualH2 = Dbetween2(p1,p2);
  CHECK(assert_equal(expectedH2,actualH2));
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST(Pose2, members)
{
  Pose2 pose;
  CHECK(pose.dim() == 3);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

