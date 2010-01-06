/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3 class
 * @author  Alireza Fathi
 */

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "Point3.h"
#include "Rot3.h"

using namespace gtsam;

Rot3 R = rodriguez(0.1,0.4,0.2);
Point3 P(0.2,0.7,-2.0);
double error = 1e-9, epsilon=0.001;

/* ************************************************************************* */
TEST( Rot3, constructor) {
  Rot3 expected(eye(3,3));
  Vector r1(3), r2(3), r3(3);
  r1(0)=1;r1(1)=0;r1(2)=0;
  r2(0)=0;r2(1)=1;r2(2)=0;
  r3(0)=0;r3(1)=0;r3(2)=1;
  Rot3 actual(r1,r2,r3);
  CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3, constructor2) {
  Matrix R = Matrix_(3,3,
		       11.,12.,13.,
		       21.,22.,23.,
		       31.,32.,33.);
  Rot3 actual(R);
  Rot3 expected(11,12,13,
		21,22,23,
		31,32,33);
  CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3, constructor3) {
  Rot3 expected(1,2,3,4,5,6,7,8,9);
  Point3 r1(1,4,7), r2(2,5,8), r3(3,6,9);
  CHECK(assert_equal(Rot3(r1,r2,r3),expected));
}

/* ************************************************************************* */
TEST( Rot3, transpose) {
  Rot3 R(1,2,3,4,5,6,7,8,9);
  Point3 r1(1,2,3), r2(4,5,6), r3(7,8,9);
  CHECK(assert_equal(R.inverse(),Rot3(r1,r2,r3)));
}

/* ************************************************************************* */
TEST( Rot3, equals) {
  CHECK(R.equals(R));
  Rot3 zero;
  CHECK(!R.equals(zero));
}

/* ************************************************************************* */
Rot3 slow_but_correct_rodriguez(const Vector& w) {
	double t = norm_2(w);
	Matrix J = skewSymmetric(w/t);
	if (t < 1e-5) return Rot3();
	Matrix R = eye(3, 3) + sin(t) * J + (1.0 - cos(t)) * (J * J);
	return R; // matrix constructor will be tripped
}

/* ************************************************************************* */
TEST( Rot3, rodriguez) {
  Rot3 R1 = rodriguez(epsilon, 0, 0);
  Vector w = Vector_(3,epsilon,0.,0.);
  Rot3 R2 = slow_but_correct_rodriguez(w);
  CHECK(assert_equal(R1,R2));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez2) {
  Vector v(3); v(0) = 0; v(1) = 1; v(2) = 0;
  Rot3 R1 = rodriguez(v, 3.14/4.0);
  Rot3 R2(0.707388,0,0.706825,
  		0,1,0,
  		-0.706825,0,0.707388);
  CHECK(assert_equal(R1,R2,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez3) {
  Vector w = Vector_(3,0.1,0.2,0.3);
  Rot3 R1 = rodriguez(w/norm_2(w), norm_2(w));
  Rot3 R2 = slow_but_correct_rodriguez(w);
  CHECK(assert_equal(R1,R2));
}

/* ************************************************************************* */
//TEST(Rot3, manifold) {
//	Rot3 t1 = rodriguez(0.1,0.4,0.2);
//	Rot3 t2 = rodriguez(0.3,0.1,0.7);
//	Rot3 origin;
//	Vector d12 = t1.log(t2);
//	CHECK(assert_equal(t2, t1.exmap(d12)));
//	CHECK(assert_equal(t2, origin.exmap(d12)*t1));
//	Vector d21 = t2.log(t1);
//	CHECK(assert_equal(t1, t2.exmap(d21)));
//	CHECK(assert_equal(t1, origin.exmap(d21)*t2));
//}

/* ************************************************************************* */
TEST( Rot3, exmap)
{
  Vector v(3);
  fill(v.begin(), v.end(), 0);
  CHECK(assert_equal(R.exmap(v), R));
}

/* ************************************************************************* */
// rotate derivatives

TEST( Rot3, Drotate1)
{
  Matrix computed = Drotate1(R, P);
  Matrix numerical = numericalDerivative21(rotate,R,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Rot3, Drotate2_DNrotate2)
{
  Matrix computed = Drotate2(R);
  Matrix numerical = numericalDerivative22(rotate,R,P);
  CHECK(assert_equal(numerical,computed,error));
}

/* ************************************************************************* */
TEST( Rot3, unrotate)
{
  Point3 w = R*P;
  CHECK(assert_equal(unrotate(R,w),P));
}

/* ************************************************************************* */
// unrotate derivatives

TEST( Rot3, Dunrotate1)
{
  Matrix computed = Dunrotate1(R, P);
  Matrix numerical = numericalDerivative21(unrotate,R,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Rot3, Dunrotate2_DNunrotate2)
{
  Matrix computed = Dunrotate2(R);
  Matrix numerical = numericalDerivative22(unrotate,R,P);
  CHECK(assert_equal(numerical,computed,error));
}

/* ************************************************************************* */
TEST( Rot3, RQ)
{
	// Try RQ on a pure rotation
	Matrix actualK; Vector actual;
  boost::tie(actualK,actual) = RQ(R.matrix());
  Vector expected = Vector_(3,0.14715, 0.385821, 0.231671);
  CHECK(assert_equal(eye(3),actualK));
  CHECK(assert_equal(expected,actual,1e-6));

  // Try using ypr call
  actual = R.ypr();
  CHECK(assert_equal(expected,actual,1e-6));

  // Try RQ to recover calibration from 3*3 sub-block of projection matrix
	Matrix K = Matrix_(3,3,
			500.0,   0.0, 320.0,
			  0.0, 500.0, 240.0,
			  0.0,   0.0,   1.0
			);
	Matrix A = K*R.matrix();
  boost::tie(actualK,actual) = RQ(A);
  CHECK(assert_equal(K,actualK));
  CHECK(assert_equal(expected,actual,1e-6));
}

/* ************************************************************************* */
int main(){ TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

