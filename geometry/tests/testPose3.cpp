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

#include <math.h>
#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

static Point3 P(0.2,0.7,-2);
static Rot3 R = Rot3::rodriguez(0.3,0,0);
static Pose3 T(R,Point3(3.5,-8.2,4.2));
static Pose3 T2(Rot3::rodriguez(0.3,0.2,0.1),Point3(3.5,-8.2,4.2));
static Pose3 T3(Rot3::rodriguez(-90, 0, 0), Point3(1, 2, 3));

/* ************************************************************************* */
TEST( Pose3, equals)
{
  Pose3 pose2 = T3;
  CHECK(T3.equals(pose2));
  Pose3 origin;
  CHECK(!T3.equals(origin));
}

/* ************************************************************************* */
TEST( Pose3, expmap_a)
{
  Pose3 id;
  Vector v(6);
  fill(v.begin(), v.end(), 0);
  v(0) = 0.3;
  CHECK(assert_equal(id.expmap(v), Pose3(R, Point3())));
#ifdef CORRECT_POSE3_EXMAP

  v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
#else
  v(3)=0.2;v(4)=0.7;v(5)=-2;
#endif
  CHECK(assert_equal(Pose3(R, P),id.expmap(v),1e-5));
}

/* ************************************************************************* */
TEST(Pose3, expmap_b)
{
  Pose3 p1(Rot3(), Point3(100, 0, 0));
  Pose3 p2 = p1.expmap(Vector_(6,0.0, 0.0, 0.1,  0.0, 0.0, 0.0));
  Pose3 expected(Rot3::rodriguez(0.0, 0.0, 0.1), Point3(100.0, 0.0, 0.0));
  CHECK(assert_equal(expected, p2));
}

#ifdef CORRECT_POSE3_EXMAP

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screw {
  double a=0.3, c=cos(a), s=sin(a), w=0.3;
	Vector xi = Vector_(6, 0.0, 0.0, w, w, 0.0, 1.0);
	Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
	Point3 expectedT(0.29552, 0.0446635, 1);
	Pose3 expected(expectedR, expectedT);
}

TEST(Pose3, expmap_c)
{
  CHECK(assert_equal(screw::expected, expm<Pose3>(screw::xi),1e-6));
  CHECK(assert_equal(screw::expected, Pose3::Expmap(screw::xi),1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(Pose3, Adjoint)
{
	Pose3 expected = T * Pose3::Expmap(screw::xi) * inverse(T);
	Vector xiprime = Adjoint(T, screw::xi);
	CHECK(assert_equal(expected, Pose3::Expmap(xiprime), 1e-6));

	Pose3 expected2 = T2 * Pose3::Expmap(screw::xi) * inverse(T2);
	Vector xiprime2 = Adjoint(T2, screw::xi);
	CHECK(assert_equal(expected2, Pose3::Expmap(xiprime2), 1e-6));

	Pose3 expected3 = T3 * Pose3::Expmap(screw::xi) * inverse(T3);
	Vector xiprime3 = Adjoint(T3, screw::xi);
	CHECK(assert_equal(expected3, Pose3::Expmap(xiprime3), 1e-6));
}

/* ************************************************************************* */
/** Agrawal06iros version */
using namespace boost::numeric::ublas;
Pose3 Agrawal06iros(const Vector& xi) {
	Vector w = vector_range<const Vector>(xi, range(0,3));
	Vector v = vector_range<const Vector>(xi, range(3,6));
	double t = norm_2(w);
	if (t < 1e-5)
		return Pose3(Rot3(), expmap<Point3> (v));
	else {
		Matrix W = skewSymmetric(w/t);
		Matrix A = eye(3) + ((1 - cos(t)) / t) * W + ((t - sin(t)) / t) * (W * W);
		return Pose3(Rot3::Expmap (w), expmap<Point3> (A * v));
	}
}

/* ************************************************************************* */
TEST(Pose3, expmaps_galore)
{
	Vector xi; Pose3 actual;
	xi = Vector_(6,0.1,0.2,0.3,0.4,0.5,0.6);
	actual = Pose3::Expmap(xi);
  CHECK(assert_equal(expm<Pose3>(xi), actual,1e-6));
  CHECK(assert_equal(Agrawal06iros(xi), actual,1e-6));
  CHECK(assert_equal(xi, logmap(actual),1e-6));

	xi = Vector_(6,0.1,-0.2,0.3,-0.4,0.5,-0.6);
	for (double theta=1.0;0.3*theta<=M_PI;theta*=2) {
		Vector txi = xi*theta;
		actual = Pose3::Expmap(txi);
		CHECK(assert_equal(expm<Pose3>(txi,30), actual,1e-6));
		CHECK(assert_equal(Agrawal06iros(txi), actual,1e-6));
		Vector log = logmap(actual);
		CHECK(assert_equal(actual, Pose3::Expmap(log),1e-6));
		CHECK(assert_equal(txi,log,1e-6)); // not true once wraps
	}

  // Works with large v as well, but expm needs 10 iterations!
	xi = Vector_(6,0.2,0.3,-0.8,100.0,120.0,-60.0);
	actual = Pose3::Expmap(xi);
  CHECK(assert_equal(expm<Pose3>(xi,10), actual,1e-5));
  CHECK(assert_equal(Agrawal06iros(xi), actual,1e-6));
  CHECK(assert_equal(xi, logmap(actual),1e-6));
}

/* ************************************************************************* */
TEST(Pose3, Adjoint_compose)
{
	// To debug derivatives of compose, assert that
	// T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
	const Pose3& T1 = T;
	Vector x = Vector_(6,0.1,0.1,0.1,0.4,0.2,0.8);
	Pose3 expected = T1 * Pose3::Expmap(x) * T2;
	Vector y = Adjoint(inverse(T2), x);
	Pose3 actual = T1 * T2 * Pose3::Expmap(y);
	CHECK(assert_equal(expected, actual, 1e-6));
}
#endif // SLOW_BUT_CORRECT_EXMAP

/* ************************************************************************* */
TEST( Pose3, compose )
{
	Matrix actual = (T2*T2).matrix();
	Matrix expected = T2.matrix()*T2.matrix();
	CHECK(assert_equal(actual,expected,1e-8));

	Matrix actualDcompose1, actualDcompose2;
	T2.compose(T2, actualDcompose1, actualDcompose2);

	Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T2, T2, 1e-4);
	CHECK(assert_equal(numericalH1,actualDcompose1,5e-5));

	Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T2, T2, 1e-4);
	CHECK(assert_equal(numericalH2,actualDcompose2));
}

/* ************************************************************************* */
TEST( Pose3, compose2 )
{
	const Pose3& T1 = T;
	Matrix actual = (T1*T2).matrix();
	Matrix expected = T1.matrix()*T2.matrix();
	CHECK(assert_equal(actual,expected,1e-8));

	Matrix actualDcompose1, actualDcompose2;
	T1.compose(T2, actualDcompose1, actualDcompose2);

	Matrix numericalH1 = numericalDerivative21(testing::compose<Pose3>, T1, T2, 1e-4);
	CHECK(assert_equal(numericalH1,actualDcompose1,5e-5));

	Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T1, T2, 1e-4);
	CHECK(assert_equal(numericalH2,actualDcompose2));
}

/* ************************************************************************* */
TEST( Pose3, inverse)
{
	Matrix actualDinverse;
	Matrix actual = T.inverse(actualDinverse).matrix();
	Matrix expected = inverse(T.matrix());
	CHECK(assert_equal(actual,expected,1e-8));

	Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T, 1e-5);
	CHECK(assert_equal(numericalH,actualDinverse));
}

/* ************************************************************************* */
TEST( Pose3, inverseDerivatives2)
{
	Rot3 R = Rot3::rodriguez(0.3,0.4,-0.5);
	Point3 t(3.5,-8.2,4.2);
	Pose3 T(R,t);

	Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T, 1e-5);
	Matrix actualDinverse;
	T.inverse(actualDinverse);
	CHECK(assert_equal(numericalH,actualDinverse,5e-5));
}

/* ************************************************************************* */
TEST( Pose3, compose_inverse)
{
	Matrix actual = (T*T.inverse()).matrix();
	Matrix expected = eye(4,4);
	CHECK(assert_equal(actual,expected,1e-8));
}

/* ************************************************************************* */
Point3 transform_from_(const Pose3& pose, const Point3& point) { return pose.transform_from(point); }
TEST( Pose3, Dtransform_from1_a)
{
	Matrix actualDtransform_from1;
	T.transform_from(P, actualDtransform_from1, boost::none);
	Matrix numerical = numericalDerivative21(transform_from_,T,P);
	CHECK(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Pose3, Dtransform_from1_b)
{
	Pose3 origin;
	Matrix actualDtransform_from1;
	origin.transform_from(P, actualDtransform_from1, boost::none);
	Matrix numerical = numericalDerivative21(transform_from_,origin,P);
	CHECK(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Pose3, Dtransform_from1_c)
{
	Point3 origin;
	Pose3 T0(R,origin);
	Matrix actualDtransform_from1;
	T0.transform_from(P, actualDtransform_from1, boost::none);
	Matrix numerical = numericalDerivative21(transform_from_,T0,P);
	CHECK(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Pose3, Dtransform_from1_d)
{
	Rot3 I;
	Point3 t0(100,0,0);
	Pose3 T0(I,t0);
	Matrix actualDtransform_from1;
	T0.transform_from(P, actualDtransform_from1, boost::none);
	//print(computed, "Dtransform_from1_d computed:");
	Matrix numerical = numericalDerivative21(transform_from_,T0,P);
	//print(numerical, "Dtransform_from1_d numerical:");
	CHECK(assert_equal(numerical,actualDtransform_from1,1e-8));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_from2)
{
	Matrix actualDtransform_from2;
	T.transform_from(P, boost::none, actualDtransform_from2);
	Matrix numerical = numericalDerivative22(transform_from_,T,P);
	CHECK(assert_equal(numerical,actualDtransform_from2,1e-8));
}

/* ************************************************************************* */
Point3 transform_to_(const Pose3& pose, const Point3& point) { return pose.transform_to(point); }
TEST( Pose3, Dtransform_to1)
{
	Matrix computed;
	T.transform_to(P, computed, boost::none);
	Matrix numerical = numericalDerivative21(transform_to_,T,P);
	CHECK(assert_equal(numerical,computed,1e-8));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_to2)
{
	Matrix computed;
	T.transform_to(P, boost::none, computed);
	Matrix numerical = numericalDerivative22(transform_to_,T,P);
	CHECK(assert_equal(numerical,computed,1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_with_derivatives)
{
	Matrix actH1, actH2;
	T.transform_to(P,actH1,actH2);
	Matrix expH1 = numericalDerivative21(transform_to_, T,P),
		   expH2 = numericalDerivative22(transform_to_, T,P);
	CHECK(assert_equal(expH1, actH1, 1e-8));
	CHECK(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transform_from_with_derivatives)
{
	Matrix actH1, actH2;
	T.transform_from(P,actH1,actH2);
	Matrix expH1 = numericalDerivative21(transform_from_, T,P),
		   expH2 = numericalDerivative22(transform_from_, T,P);
	CHECK(assert_equal(expH1, actH1, 1e-8));
	CHECK(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_translate)
{
		Point3 actual = Pose3(Rot3(), Point3(1, 2, 3)).transform_to(Point3(10.,20.,30.));
		Point3 expected(9.,18.,27.);
		CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_rotate)
{
		Pose3 transform(Rot3::rodriguez(0,0,-1.570796), Point3());
		Point3 actual = transform.transform_to(Point3(2,1,10));
		Point3 expected(-1,2,10);
		CHECK(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST( Pose3, transform_to)
{
		Pose3 transform(Rot3::rodriguez(0,0,-1.570796), Point3(2,4, 0));
		Point3 actual = transform.transform_to(Point3(3,2,10));
		Point3 expected(2,1,10);
		CHECK(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST( Pose3, transform_from)
{
		Point3 actual = T3.transform_from(Point3());
		Point3 expected = Point3(1.,2.,3.);
		CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transform_roundtrip)
{
		Point3 actual = T3.transform_from(T3.transform_to(Point3(12., -0.11,7.0)));
		Point3 expected(12., -0.11,7.0);
		CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_origin)
{
		// transform to origin
		Pose3 actual = T3.transform_to(Pose3());
		CHECK(assert_equal(T3, actual, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_itself)
{
		// transform to itself
		Pose3 actual = T3.transform_to(T3);
		CHECK(assert_equal(Pose3(), actual, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_translation)
{
		// transform translation only
		Rot3 r = Rot3::rodriguez(-1.570796,0,0);
		Pose3 pose2(r, Point3(21.,32.,13.));
		Pose3 actual = pose2.transform_to(Pose3(Rot3(), Point3(1,2,3)));
		Pose3 expected(r, Point3(20.,30.,10.));
		CHECK(assert_equal(expected, actual, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_simple_rotate)
{
		// transform translation only
		Rot3 r = Rot3::rodriguez(0,0,-1.570796);
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
		Rot3 r = Rot3::rodriguez(0,0,-1.570796); //-90 degree yaw
		Rot3 r2 = Rot3::rodriguez(0,0,0.698131701); //40 degree yaw
		Pose3 pose2(r2, Point3(21.,32.,13.));
		Pose3 transform(r, Point3(1,2,3));
		Pose3 actual = pose2.transform_to(transform);
		Pose3 expected(Rot3::rodriguez(0,0,2.26892803), Point3(-30.,20.,10.));
		CHECK(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST(Pose3, manifold)
{
	//cout << "manifold" << endl;
	Pose3 t1 = T;
	Pose3 t2 = T3;
	Pose3 origin;
	Vector d12 = t1.logmap(t2);
	CHECK(assert_equal(t2, t1.expmap(d12)));
	// todo: richard - commented out because this tests for "compose-style" (new) expmap
	// CHECK(assert_equal(t2, expmap(origin,d12)*t1));
	Vector d21 = t2.logmap(t1);
	CHECK(assert_equal(t1, t2.expmap(d21)));
	// todo: richard - commented out because this tests for "compose-style" (new) expmap
	// CHECK(assert_equal(t1, expmap(origin,d21)*t2));

	// Check that log(t1,t2)=-log(t2,t1) - this holds even for incorrect expmap :-)
	CHECK(assert_equal(d12,-d21));

#ifdef CORRECT_POSE3_EXMAP


	// todo: Frank - Below only works for correct "Agrawal06iros style expmap
	// lines in canonical coordinates correspond to Abelian subgroups in SE(3)
	 Vector d = Vector_(6,0.1,0.2,0.3,0.4,0.5,0.6);
	// exp(-d)=inverse(exp(d))
	 CHECK(assert_equal(Pose3::Expmap(-d),inverse(Pose3::Expmap(d))));
	// exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
	 Pose3 T2 = Pose3::Expmap(2*d);
	 Pose3 T3 = Pose3::Expmap(3*d);
	 Pose3 T5 = Pose3::Expmap(5*d);
	 CHECK(assert_equal(T5,T2*T3));
	 CHECK(assert_equal(T5,T3*T2));

#endif
}

/* ************************************************************************* */
TEST( Pose3, between )
{
	Pose3 expected = T2.inverse() * T3;
	Matrix actualDBetween1,actualDBetween2;
	Pose3 actual = T2.between(T3, actualDBetween1,actualDBetween2);
	CHECK(assert_equal(expected,actual));

	Matrix numericalH1 = numericalDerivative21(testing::between<Pose3> , T2, T3, 1e-4);
	CHECK(assert_equal(numericalH1,actualDBetween1,5e-5));

	Matrix numericalH2 = numericalDerivative22(testing::between<Pose3> , T2, T3, 1e-4);
	CHECK(assert_equal(numericalH2,actualDBetween2));
}

/* ************************************************************************* */
int main(){ TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
