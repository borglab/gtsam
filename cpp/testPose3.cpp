/**
 * @file   testPose3.cpp
 * @brief  Unit tests for Pose3 class
 */


#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "Pose3.h"

using namespace gtsam;

Rot3 R = rodriguez(0.3,0,0);
Point3 t(3.5,-8.2,4.2);
Pose3 T(R,t);
Point3 P(0.2,0.7,-2);
Rot3 r1 = rodriguez(-90, 0, 0);
Pose3 pose1(r1, Point3(1, 2, 3));
double error = 1e-8;

#define PI 3.14159265358979323846


/* ************************************************************************* */
TEST( Pose3, equals)
{
  Pose3 pose2 = pose1;
  CHECK(pose1.equals(pose2));
  Pose3 origin;
  CHECK(!pose1.equals(origin));
}

/* ************************************************************************* */
TEST( Pose3, expmap_a)
{
  Pose3 id;
  Vector v(6);
  fill(v.begin(), v.end(), 0);
  v(0) = 0.3;
  CHECK(assert_equal(expmap(id,v), Pose3(R, Point3())));
  v(3)=0.2;v(4)=0.7;v(5)=-2;
  CHECK(assert_equal(expmap(id,v), Pose3(R, P)));
}

TEST(Pose3, expmap_b)
{
  Pose3 p1(Rot3(), Point3(100, 0, 0));
  Pose3 p2 = expmap(p1, Vector_(6,
      0.0, 0.0, 0.1,  0.0, 0.0, 0.0));
  Pose3 expected(rodriguez(0.0, 0.0, 0.1), Point3(100.0, 0.0, 0.0));
  CHECK(assert_equal(expected, p2));
}

/* ************************************************************************* */
TEST( Pose3, compose )
{
	Rot3 R = rodriguez(0.3,0.2,0.1);
	Point3 t(3.5,-8.2,4.2);
	Pose3 T(R,t);

  Matrix actual = (T*T).matrix();
  Matrix expected = T.matrix()*T.matrix();
  CHECK(assert_equal(actual,expected,error));

	Matrix numericalH1 = numericalDerivative21<Pose3,Pose3,Pose3>(compose, T, T, 1e-5);
	Matrix actualH1 = Dcompose1(T, T);
	CHECK(assert_equal(numericalH1,actualH1));

	Matrix actualH2 = Dcompose2(T, T);
	Matrix numericalH2 = numericalDerivative22<Pose3,Pose3,Pose3>(compose, T, T, 1e-5);
	CHECK(assert_equal(numericalH2,actualH2));}

/* ************************************************************************* */
TEST( Pose3, inverse)
{
  Matrix actual = inverse(T).matrix();
  Matrix expected = inverse(T.matrix());
  CHECK(assert_equal(actual,expected,error));

	Matrix numericalH = numericalDerivative11<Pose3,Pose3>(inverse, T, 1e-5);
	Matrix actualH = Dinverse(T);
	CHECK(assert_equal(numericalH,actualH));
}

/* ************************************************************************* */
TEST( Pose3, compose_inverse)
{
  Matrix actual = (T*inverse(T)).matrix();
  Matrix expected = eye(4,4);
  CHECK(assert_equal(actual,expected,error));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_from1_a)
{
  Matrix computed = Dtransform_from1(T, P);
  Matrix numerical = numericalDerivative21(transform_from,T,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Pose3, Dtransform_from1_b)
{
	Pose3 origin;
  Matrix computed = Dtransform_from1(origin, P);
  Matrix numerical = numericalDerivative21(transform_from,origin,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Pose3, Dtransform_from1_c)
{
	Point3 origin;
	Pose3 T0(R,origin);
  Matrix computed = Dtransform_from1(T0, P);
  Matrix numerical = numericalDerivative21(transform_from,T0,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Pose3, Dtransform_from1_d)
{
	Rot3 I;
	Point3 t0(100,0,0);
	Pose3 T0(I,t0);
  Matrix computed = Dtransform_from1(T0, P);
  //print(computed, "Dtransform_from1_d computed:");
  Matrix numerical = numericalDerivative21(transform_from,T0,P);
  //print(numerical, "Dtransform_from1_d numerical:");
  CHECK(assert_equal(numerical,computed,error));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_from2)
{
  Matrix computed = Dtransform_from2(T);
  Matrix numerical = numericalDerivative22(transform_from,T,P);
  CHECK(assert_equal(numerical,computed,error));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_to1)
{
  Matrix computed = Dtransform_to1(T, P);
  Matrix numerical = numericalDerivative21(transform_to,T,P);
  CHECK(assert_equal(numerical,computed,error));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_to2)
{
  Matrix computed = Dtransform_to2(T,P);
  Matrix numerical = numericalDerivative22(transform_to,T,P);
  CHECK(assert_equal(numerical,computed,error));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_translate)
{
		Point3 actual = transform_to(Pose3(Rot3(), Point3(1, 2, 3)), Point3(10.,20.,30.));
		Point3 expected(9.,18.,27.);
		CHECK(assert_equal(expected, actual)); 
}

/* ************************************************************************* */
TEST( Pose3, transform_to_rotate)
{
		Pose3 transform(rodriguez(0,0,-1.570796), Point3()); 
		Point3 actual = transform_to(transform, Point3(2,1,10));
		Point3 expected(-1,2,10);
		CHECK(assert_equal(expected, actual, 0.001)); 
}

/* ************************************************************************* */
TEST( Pose3, transform_to)
{
		Pose3 transform(rodriguez(0,0,-1.570796), Point3(2,4, 0)); 
		Point3 actual = transform_to(transform, Point3(3,2,10));
		Point3 expected(2,1,10);
		CHECK(assert_equal(expected, actual, 0.001)); 
}

/* ************************************************************************* */
TEST( Pose3, transform_from)
{
		Point3 actual = transform_from(pose1, Point3());
		Point3 expected = Point3(1.,2.,3.);
		CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transform_roundtrip)
{
		Point3 actual = transform_from(pose1, transform_to(pose1, Point3(12., -0.11,7.0)));
		Point3 expected(12., -0.11,7.0);
		CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_origin)
{
		// transform to origin
		Pose3 actual = pose1.transform_to(Pose3());
		CHECK(assert_equal(pose1, actual, error));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_itself)
{
		// transform to itself
		Pose3 actual = pose1.transform_to(pose1);
		CHECK(assert_equal(Pose3(), actual, error));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_translation)
{
		// transform translation only
		Rot3 r = rodriguez(-1.570796,0,0);
		Pose3 pose2(r, Point3(21.,32.,13.)); 
		Pose3 actual = pose2.transform_to(Pose3(Rot3(), Point3(1,2,3)));
		Pose3 expected(r, Point3(20.,30.,10.));  
		CHECK(assert_equal(expected, actual, error)); 
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_simple_rotate)
{
		// transform translation only
		Rot3 r = rodriguez(0,0,-1.570796);
		Pose3 pose2(r, Point3(21.,32.,13.)); 
		Pose3 transform(r, Point3(1,2,3));
		Pose3 actual = pose2.transform_to(transform);
		Pose3 expected(Rot3(), Point3(-30.,20.,10.)); 
		CHECK(assert_equal(expected, actual, 0.001)); 
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to)
{
		// transform to
		Rot3 r = rodriguez(0,0,-1.570796); //-90 degree yaw
		Rot3 r2 = rodriguez(0,0,0.698131701); //40 degree yaw
		Pose3 pose2(r2, Point3(21.,32.,13.)); 
		Pose3 transform(r, Point3(1,2,3));
		Pose3 actual = pose2.transform_to(transform);
		Pose3 expected(rodriguez(0,0,2.26892803), Point3(-30.,20.,10.)); 
		CHECK(assert_equal(expected, actual, 0.001));  
}

/* ************************************************************************* */
TEST( Pose3, composeTransform )
{
	// known transform
	Rot3 R1 = rodriguez(0, 0, -1.570796);
	Pose3 expected(R1, Point3(1, 2, 3));

	// current
	Rot3 R2 = rodriguez(0, 0, 0.698131701);
	Pose3 current(R2, Point3(21., 32., 13.));

	// target
	Pose3 target(rodriguez(0, 0, 2.26892803), Point3(-30., 20., 10.));

	// calculate transform
	// todo: which should this be?
	//Pose3 actual = compose(current, target);
	Pose3 actual = between<Pose3> (target, current);

	//verify
	CHECK(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST(Pose3, manifold)
{
	//cout << "manifold" << endl;
	Pose3 t1 = T;
	Pose3 t2 = pose1;
	Pose3 origin;
	Vector d12 = logmap(t1, t2);
	CHECK(assert_equal(t2, expmap(t1,d12)));
	// todo: richard - commented out because this tests for "compose-style" (new) expmap
	// CHECK(assert_equal(t2, expmap(origin,d12)*t1));
	Vector d21 = logmap(t2, t1);
	CHECK(assert_equal(t1, expmap(t2,d21)));
	// todo: richard - commented out because this tests for "compose-style" (new) expmap
	// CHECK(assert_equal(t1, expmap(origin,d21)*t2));

	// Check that log(t1,t2)=-log(t2,t1) - this holds even for incorrect expmap :-)
	CHECK(assert_equal(d12,-d21));

#ifdef SLOW_BUT_CORRECT_EXPMAP

	// todo: Frank - Below only works for correct "Agrawal06iros style expmap
	// lines in canonical coordinates correspond to Abelian subgroups in SE(3)
	 Vector d = Vector_(6,0.1,0.2,0.3,0.4,0.5,0.6);
	// exp(-d)=inverse(exp(d))
	 CHECK(assert_equal(expmap<Pose3>(-d),inverse(expmap<Pose3>(d))));
	// exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
	 Pose3 T2 = expmap<Pose3>(2*d);
	 Pose3 T3 = expmap<Pose3>(3*d);
	 Pose3 T5 = expmap<Pose3>(5*d);
	 CHECK(assert_equal(T5,T2*T3));
	 CHECK(assert_equal(T5,T3*T2));

#endif
}

/* ************************************************************************* */
TEST( Pose3, between )
{
	Rot3 R = rodriguez(0.3,0.2,0.1);
	Point3 t(3.5,-8.2,4.2);
	Pose3 T(R,t);

	Pose3 expected = pose1 * inverse(T);
	Pose3 actual = between(T, pose1);
	CHECK(assert_equal(expected,actual));

	Matrix numericalH1 = numericalDerivative21(between<Pose3> , T, pose1, 1e-5);
	Matrix actualH1 = Dbetween1(T, pose1);
	CHECK(assert_equal(numericalH1,actualH1)); // chain rule does not work ??

	Matrix actualH2 = Dbetween2(T, pose1);
	Matrix numericalH2 = numericalDerivative22(between<Pose3> , T, pose1, 1e-5);
	CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
int main(){ TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

