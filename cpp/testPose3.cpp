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
double error = 1e-9;

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
TEST( Pose3, exmap)
{
  Pose3 id;
  Vector v(6);
  fill(v.begin(), v.end(), 0);
  v(0) = 0.3;
  CHECK(assert_equal(id.exmap(v), Pose3(R, Point3())));
  v(3)=0.2;v(4)=0.7;v(5)=-2;
  CHECK(assert_equal(id.exmap(v), Pose3(R, P)));
}

/* ************************************************************************* */
TEST( Pose3, compose )
{
  Matrix actual = (T*T).matrix();
  Matrix expected = T.matrix()*T.matrix();
  CHECK(assert_equal(actual,expected,error));
}

/* ************************************************************************* */
TEST( Pose3, inverse)
{
  Matrix actual = T.inverse().matrix();
  Matrix expected = inverse(T.matrix());
  CHECK(assert_equal(actual,expected,error));
}

/* ************************************************************************* */
TEST( Pose3, compose_inverse)
{
  Matrix actual = (T*T.inverse()).matrix();
  Matrix expected = eye(4,4);
  CHECK(assert_equal(actual,expected,error));
}

/* ************************************************************************* */
// transform derivatives

TEST( Pose3, Dtransform_from1)
{
  Matrix computed = Dtransform_from1(T, P);
  Matrix numerical = numericalDerivative21(transform_from,T,P);
  CHECK(assert_equal(numerical,computed,error));
}


TEST( Pose3, Dtransform_from2)
{
  Matrix computed = Dtransform_from2(T);
  Matrix numerical = numericalDerivative22(transform_from,T,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Pose3, Dtransform_to1)
{
  Matrix computed = Dtransform_to1(T, P);
  Matrix numerical = numericalDerivative21(transform_to,T,P);
  CHECK(assert_equal(numerical,computed,error));
}

TEST( Pose3, Dtransform_to2)
{
  Matrix computed = Dtransform_to2(T);
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
TEST( Pose3, transform_to_rotate)
{
		Pose3 transform(rodriguez(0,0,-1.570796), Point3()); 
		Point3 actual = transform_to(transform, Point3(2,1,10));
		Point3 expected(-1,2,10);
		CHECK(assert_equal(expected, actual, 0.001)); 
}
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
		Pose3 actual = pose1.transformPose_to(Pose3());
		CHECK(assert_equal(pose1, actual, error));
}
TEST( Pose3, transformPose_to_itself)
{
		// transform to itself
		Pose3 actual = pose1.transformPose_to(pose1);
		CHECK(assert_equal(Pose3(), actual, error));
}
TEST( Pose3, transformPose_to_translation)
{
		// transform translation only
		Rot3 r = rodriguez(-1.570796,0,0);
		Pose3 pose2(r, Point3(21.,32.,13.)); 
		Pose3 actual = pose2.transformPose_to(Pose3(Rot3(), Point3(1,2,3)));
		Pose3 expected(r, Point3(20.,30.,10.));  
		CHECK(assert_equal(expected, actual, error)); 
}
TEST( Pose3, transformPose_to_simple_rotate)
{
		// transform translation only
		Rot3 r = rodriguez(0,0,-1.570796);
		Pose3 pose2(r, Point3(21.,32.,13.)); 
		Pose3 transform(r, Point3(1,2,3));
		Pose3 actual = pose2.transformPose_to(transform);
		Pose3 expected(Rot3(), Point3(-30.,20.,10.)); 
		CHECK(assert_equal(expected, actual, 0.001)); 
}
TEST( Pose3, transformPose_to)
{
		// transform to
		Rot3 r = rodriguez(0,0,-1.570796); //-90 degree yaw
		Rot3 r2 = rodriguez(0,0,0.698131701); //40 degree yaw
		Pose3 pose2(r2, Point3(21.,32.,13.)); 
		Pose3 transform(r, Point3(1,2,3));
		Pose3 actual = pose2.transformPose_to(transform);
		Pose3 expected(rodriguez(0,0,2.26892803), Point3(-30.,20.,10.)); 
		CHECK(assert_equal(expected, actual, 0.001));  
}

/* ************************************************************************* */
TEST( Pose3, composeTransform )
{
		// known transform
		Rot3 r = rodriguez(0,0,-1.570796);
		Pose3 exp_transform(r, Point3(1,2,3));
		
		// current
		Rot3 r2 = rodriguez(0,0,0.698131701); 
		Pose3 current(r2, Point3(21.,32.,13.));
		
		// target 
		Pose3 target(rodriguez(0,0,2.26892803), Point3(-30.,20.,10.)); 
		
		// calculate transform
		Pose3 actual = composeTransform(current, target);
		
		//verify
		CHECK(assert_equal(exp_transform, actual, 0.001));
}

/* ************************************************************************* */
int main(){ TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

