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
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Pose3)
GTSAM_CONCEPT_LIE_INST(Pose3)

static Point3 P(0.2,0.7,-2);
static Rot3 R = Rot3::rodriguez(0.3,0,0);
static Pose3 T(R,Point3(3.5,-8.2,4.2));
static Pose3 T2(Rot3::rodriguez(0.3,0.2,0.1),Point3(3.5,-8.2,4.2));
static Pose3 T3(Rot3::rodriguez(-90, 0, 0), Point3(1, 2, 3));
const double tol=1e-5;

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
	Pose3 expected(Rot3::rodriguez(0,0,3),Point3(1,2,0));
  Pose2 pose2(1,2,3);
  EXPECT(assert_equal(expected,Pose3(pose2)));
}

/* ************************************************************************* */
TEST( Pose3, retract_first_order)
{
  Pose3 id;
  Vector v = zero(6);
  v(0) = 0.3;
  EXPECT(assert_equal(Pose3(R, Point3()), id.retract(v, Pose3::FIRST_ORDER),1e-2));
  v(3)=0.2;v(4)=0.7;v(5)=-2;
  EXPECT(assert_equal(Pose3(R, P),id.retract(v, Pose3::FIRST_ORDER),1e-2));
}

/* ************************************************************************* */
TEST( Pose3, retract_expmap)
{
  Pose3 id;
  Vector v = zero(6);
  v(0) = 0.3;
  EXPECT(assert_equal(Pose3(R, Point3()), id.retract(v, Pose3::EXPMAP),1e-2));
  v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
  EXPECT(assert_equal(Pose3(R, P),id.retract(v, Pose3::EXPMAP),1e-2));
}

/* ************************************************************************* */
TEST( Pose3, expmap_a_full)
{
  Pose3 id;
  Vector v = zero(6);
  v(0) = 0.3;
  EXPECT(assert_equal(expmap_default<Pose3>(id, v), Pose3(R, Point3())));
  v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
  EXPECT(assert_equal(Pose3(R, P),expmap_default<Pose3>(id, v),1e-5));
}

/* ************************************************************************* */
TEST( Pose3, expmap_a_full2)
{
  Pose3 id;
  Vector v = zero(6);
  v(0) = 0.3;
  EXPECT(assert_equal(expmap_default<Pose3>(id, v), Pose3(R, Point3())));
  v(3)=0.2;v(4)=0.394742;v(5)=-2.08998;
  EXPECT(assert_equal(Pose3(R, P),expmap_default<Pose3>(id, v),1e-5));
}

/* ************************************************************************* */
TEST(Pose3, expmap_b)
{
  Pose3 p1(Rot3(), Point3(100, 0, 0));
  Pose3 p2 = p1.retract(Vector_(6,0.0, 0.0, 0.1,  0.0, 0.0, 0.0));
  Pose3 expected(Rot3::rodriguez(0.0, 0.0, 0.1), Point3(100.0, 0.0, 0.0));
  EXPECT(assert_equal(expected, p2,1e-2));
}

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screw {
  double a=0.3, c=cos(a), s=sin(a), w=0.3;
	Vector xi = Vector_(6, 0.0, 0.0, w, w, 0.0, 1.0);
	Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
	Point3 expectedT(0.29552, 0.0446635, 1);
	Pose3 expected(expectedR, expectedT);
}

/* ************************************************************************* */
// Checks correct exponential map (Expmap) with brute force matrix exponential
TEST(Pose3, expmap_c_full)
{
  EXPECT(assert_equal(screw::expected, expm<Pose3>(screw::xi),1e-6));
  EXPECT(assert_equal(screw::expected, Pose3::Expmap(screw::xi),1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(Pose3, Adjoint_full)
{
	Pose3 expected = T * Pose3::Expmap(screw::xi) * T.inverse();
	Vector xiprime = T.adjoint(screw::xi);
	EXPECT(assert_equal(expected, Pose3::Expmap(xiprime), 1e-6));

	Pose3 expected2 = T2 * Pose3::Expmap(screw::xi) * T2.inverse();
	Vector xiprime2 = T2.adjoint(screw::xi);
	EXPECT(assert_equal(expected2, Pose3::Expmap(xiprime2), 1e-6));

	Pose3 expected3 = T3 * Pose3::Expmap(screw::xi) * T3.inverse();
	Vector xiprime3 = T3.adjoint(screw::xi);
	EXPECT(assert_equal(expected3, Pose3::Expmap(xiprime3), 1e-6));
}

/* ************************************************************************* */
/** Agrawal06iros version of exponential map */
Pose3 Agrawal06iros(const Vector& xi) {
	Vector w = xi.head(3);
	Vector v = xi.tail(3);
	double t = norm_2(w);
	if (t < 1e-5)
		return Pose3(Rot3(), Point3::Expmap(v));
	else {
		Matrix W = skewSymmetric(w/t);
		Matrix A = eye(3) + ((1 - cos(t)) / t) * W + ((t - sin(t)) / t) * (W * W);
		return Pose3(Rot3::Expmap (w), Point3::Expmap(A * v));
	}
}

/* ************************************************************************* */
TEST(Pose3, expmaps_galore_full)
{
	Vector xi; Pose3 actual;
	xi = Vector_(6,0.1,0.2,0.3,0.4,0.5,0.6);
	actual = Pose3::Expmap(xi);
  EXPECT(assert_equal(expm<Pose3>(xi), actual,1e-6));
  EXPECT(assert_equal(Agrawal06iros(xi), actual,1e-6));
  EXPECT(assert_equal(xi, Pose3::Logmap(actual),1e-6));

	xi = Vector_(6,0.1,-0.2,0.3,-0.4,0.5,-0.6);
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
	xi = Vector_(6,0.2,0.3,-0.8,100.0,120.0,-60.0);
	actual = Pose3::Expmap(xi);
  EXPECT(assert_equal(expm<Pose3>(xi,10), actual,1e-5));
  EXPECT(assert_equal(Agrawal06iros(xi), actual,1e-6));
  EXPECT(assert_equal(xi, Pose3::Logmap(actual),1e-6));
}

/* ************************************************************************* */
TEST(Pose3, Adjoint_compose_full)
{
	// To debug derivatives of compose, assert that
	// T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
	const Pose3& T1 = T;
	Vector x = Vector_(6,0.1,0.1,0.1,0.4,0.2,0.8);
	Pose3 expected = T1 * Pose3::Expmap(x) * T2;
	Vector y = T2.inverse().adjoint(x);
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
	EXPECT(assert_equal(T2.inverse().adjointMap(),actualDcompose1,5e-3));

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
	EXPECT(assert_equal(T2.inverse().adjointMap(),actualDcompose1,5e-3));

	Matrix numericalH2 = numericalDerivative22(testing::compose<Pose3>, T1, T2);
	EXPECT(assert_equal(numericalH2,actualDcompose2,1e-5));
}

/* ************************************************************************* */
TEST( Pose3, inverse)
{
	Matrix actualDinverse;
	Matrix actual = T.inverse(actualDinverse).matrix();
	Matrix expected = inverse(T.matrix());
	EXPECT(assert_equal(actual,expected,1e-8));

	Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
	EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
	EXPECT(assert_equal(-T.adjointMap(),actualDinverse,5e-3));
}

/* ************************************************************************* */
TEST( Pose3, inverseDerivatives2)
{
	Rot3 R = Rot3::rodriguez(0.3,0.4,-0.5);
	Point3 t(3.5,-8.2,4.2);
	Pose3 T(R,t);

	Matrix numericalH = numericalDerivative11(testing::inverse<Pose3>, T);
	Matrix actualDinverse;
	T.inverse(actualDinverse);
	EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
	EXPECT(assert_equal(-T.adjointMap(),actualDinverse,5e-3));
}

/* ************************************************************************* */
TEST( Pose3, compose_inverse)
{
	Matrix actual = (T*T.inverse()).matrix();
	Matrix expected = eye(4,4);
	EXPECT(assert_equal(actual,expected,1e-8));
}

/* ************************************************************************* */
Point3 transform_from_(const Pose3& pose, const Point3& point) { return pose.transform_from(point); }
TEST( Pose3, Dtransform_from1_a)
{
	Matrix actualDtransform_from1;
	T.transform_from(P, actualDtransform_from1, boost::none);
	Matrix numerical = numericalDerivative21(transform_from_,T,P);
	EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Pose3, Dtransform_from1_b)
{
	Pose3 origin;
	Matrix actualDtransform_from1;
	origin.transform_from(P, actualDtransform_from1, boost::none);
	Matrix numerical = numericalDerivative21(transform_from_,origin,P);
	EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Pose3, Dtransform_from1_c)
{
	Point3 origin;
	Pose3 T0(R,origin);
	Matrix actualDtransform_from1;
	T0.transform_from(P, actualDtransform_from1, boost::none);
	Matrix numerical = numericalDerivative21(transform_from_,T0,P);
	EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
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
	EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_from2)
{
	Matrix actualDtransform_from2;
	T.transform_from(P, boost::none, actualDtransform_from2);
	Matrix numerical = numericalDerivative22(transform_from_,T,P);
	EXPECT(assert_equal(numerical,actualDtransform_from2,1e-8));
}

/* ************************************************************************* */
Point3 transform_to_(const Pose3& pose, const Point3& point) { return pose.transform_to(point); }
TEST( Pose3, Dtransform_to1)
{
	Matrix computed;
	T.transform_to(P, computed, boost::none);
	Matrix numerical = numericalDerivative21(transform_to_,T,P);
	EXPECT(assert_equal(numerical,computed,1e-8));
}

/* ************************************************************************* */
TEST( Pose3, Dtransform_to2)
{
	Matrix computed;
	T.transform_to(P, boost::none, computed);
	Matrix numerical = numericalDerivative22(transform_to_,T,P);
	EXPECT(assert_equal(numerical,computed,1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_with_derivatives)
{
	Matrix actH1, actH2;
	T.transform_to(P,actH1,actH2);
	Matrix expH1 = numericalDerivative21(transform_to_, T,P),
		   expH2 = numericalDerivative22(transform_to_, T,P);
	EXPECT(assert_equal(expH1, actH1, 1e-8));
	EXPECT(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transform_from_with_derivatives)
{
	Matrix actH1, actH2;
	T.transform_from(P,actH1,actH2);
	Matrix expH1 = numericalDerivative21(transform_from_, T,P),
		   expH2 = numericalDerivative22(transform_from_, T,P);
	EXPECT(assert_equal(expH1, actH1, 1e-8));
	EXPECT(assert_equal(expH2, actH2, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_translate)
{
		Point3 actual = Pose3(Rot3(), Point3(1, 2, 3)).transform_to(Point3(10.,20.,30.));
		Point3 expected(9.,18.,27.);
		EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transform_to_rotate)
{
		Pose3 transform(Rot3::rodriguez(0,0,-1.570796), Point3());
		Point3 actual = transform.transform_to(Point3(2,1,10));
		Point3 expected(-1,2,10);
		EXPECT(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST( Pose3, transform_to)
{
		Pose3 transform(Rot3::rodriguez(0,0,-1.570796), Point3(2,4, 0));
		Point3 actual = transform.transform_to(Point3(3,2,10));
		Point3 expected(2,1,10);
		EXPECT(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST( Pose3, transform_from)
{
		Point3 actual = T3.transform_from(Point3());
		Point3 expected = Point3(1.,2.,3.);
		EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transform_roundtrip)
{
		Point3 actual = T3.transform_from(T3.transform_to(Point3(12., -0.11,7.0)));
		Point3 expected(12., -0.11,7.0);
		EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_origin)
{
		// transform to origin
		Pose3 actual = T3.transform_to(Pose3());
		EXPECT(assert_equal(T3, actual, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_itself)
{
		// transform to itself
		Pose3 actual = T3.transform_to(T3);
		EXPECT(assert_equal(Pose3(), actual, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3, transformPose_to_translation)
{
		// transform translation only
		Rot3 r = Rot3::rodriguez(-1.570796,0,0);
		Pose3 pose2(r, Point3(21.,32.,13.));
		Pose3 actual = pose2.transform_to(Pose3(Rot3(), Point3(1,2,3)));
		Pose3 expected(r, Point3(20.,30.,10.));
		EXPECT(assert_equal(expected, actual, 1e-8));
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
		EXPECT(assert_equal(expected, actual, 0.001));
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
		EXPECT(assert_equal(expected, actual, 0.001));
}

/* ************************************************************************* */
TEST(Pose3, localCoordinates_first_order)
{
	Vector d12 = repeat(6,0.1);
	Pose3 t1 = T, t2 = t1.retract(d12, Pose3::FIRST_ORDER);
	EXPECT(assert_equal(d12, t1.localCoordinates(t2, Pose3::FIRST_ORDER)));
}

/* ************************************************************************* */
TEST(Pose3, localCoordinates_expmap)
{
  Vector d12 = repeat(6,0.1);
  Pose3 t1 = T, t2 = t1.retract(d12, Pose3::EXPMAP);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2, Pose3::EXPMAP)));
}

/* ************************************************************************* */
TEST(Pose3, manifold_first_order)
{
	Pose3 t1 = T;
	Pose3 t2 = T3;
	Pose3 origin;
	Vector d12 = t1.localCoordinates(t2, Pose3::FIRST_ORDER);
	EXPECT(assert_equal(t2, t1.retract(d12, Pose3::FIRST_ORDER)));
	Vector d21 = t2.localCoordinates(t1, Pose3::FIRST_ORDER);
	EXPECT(assert_equal(t1, t2.retract(d21, Pose3::FIRST_ORDER)));
}

/* ************************************************************************* */
TEST(Pose3, manifold_expmap)
{
  Pose3 t1 = T;
  Pose3 t2 = T3;
  Pose3 origin;
  Vector d12 = t1.localCoordinates(t2, Pose3::EXPMAP);
  EXPECT(assert_equal(t2, t1.retract(d12, Pose3::EXPMAP)));
  Vector d21 = t2.localCoordinates(t1, Pose3::EXPMAP);
  EXPECT(assert_equal(t1, t2.retract(d21, Pose3::EXPMAP)));

  // Check that log(t1,t2)=-log(t2,t1)
  EXPECT(assert_equal(d12,-d21));
}

/* ************************************************************************* */
TEST(Pose3, subgroups)
{
	// Frank - Below only works for correct "Agrawal06iros style expmap
	// lines in canonical coordinates correspond to Abelian subgroups in SE(3)
	 Vector d = Vector_(6,0.1,0.2,0.3,0.4,0.5,0.6);
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
Pose3 x1, x2(Rot3::ypr(0.0, 0.0, 0.0), l2), x3(Rot3::ypr(M_PI/4.0, 0.0, 0.0), l2);
Pose3
		xl1(Rot3::ypr(0.0, 0.0, 0.0), Point3(1, 0, 0)),
		xl2(Rot3::ypr(0.0, 1.0, 0.0), Point3(1, 1, 0)),
		xl3(Rot3::ypr(1.0, 0.0, 0.0), Point3(2, 2, 0)),
		xl4(Rot3::ypr(0.0, 0.0, 1.0), Point3(1, 4,-4));

/* ************************************************************************* */
LieVector range_proxy(const Pose3& pose, const Point3& point) {
	return LieVector(pose.range(point));
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
LieVector range_pose_proxy(const Pose3& pose, const Pose3& point) {
	return LieVector(pose.range(point));
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
TEST( Pose3, unicycle )
{
	// velocity in X should be X in inertial frame, rather than global frame
	Vector x_step = delta(6,3,1.0);
	EXPECT(assert_equal(Pose3(Rot3::ypr(0,0,0), l1), expmap_default<Pose3>(x1, x_step), tol));
	EXPECT(assert_equal(Pose3(Rot3::ypr(0,0,0), Point3(2,1,0)), expmap_default<Pose3>(x2, x_step), tol));
	EXPECT(assert_equal(Pose3(Rot3::ypr(M_PI/4.0,0,0), Point3(2,2,0)), expmap_default<Pose3>(x3, sqrt(2.0) * x_step), tol));
}

/* ************************************************************************* */
int main(){ TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
