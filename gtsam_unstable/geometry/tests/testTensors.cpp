/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testTensors.cpp
 * @brief try tensor expressions based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 9, 2010
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/geometry/tensors.h>
#include <gtsam_unstable/geometry/tensorInterface.h>
#include <gtsam_unstable/geometry/projectiveGeometry.h>

using namespace std;
using namespace gtsam;
using namespace tensors;

/* ************************************************************************* */
// Indices

Index<3, 'a'> a, _a;
Index<3, 'b'> b, _b;
Index<3, 'c'> c, _c;

Index<4, 'A'> A;
Index<4, 'B'> B;

/* ************************************************************************* */
// Tensor1
/* ************************************************************************* */
TEST(Tensor1, Basics)
{
	// you can create 1-tensors corresponding to 2D homogeneous points
	// using the function point2h in projectiveGeometry.*
	Point2h p = point2h(1, 2, 3), q = point2h(2, 4, 6);

	// equality tests always take tensor expressions, not tensors themselves
	// the difference is that a tensor expression has indices
	CHECK(p(a)==p(a))
	CHECK(assert_equality(p(a),p(a)))
	CHECK(assert_equality(p(a)*2,q(a)))
	CHECK(assert_equivalent(p(a),q(a))) // projectively equivalent

	// and you can take a norm, typically for normalization to the sphere
	DOUBLES_EQUAL(sqrt(14),norm(p(a)),1e-9)
}

/* ************************************************************************* */
TEST( Tensor1, Incidence2D)
{
	// 2D lines are created with line2h
	Line2h l = line2h(-13, 5, 1);
	Point2h p = point2h(1, 2, 3), q = point2h(2, 5, 1);

	// Incidence between a line and a point is checked with simple contraction
	// It does not matter which index you use, but it has to be of dimension 3
	DOUBLES_EQUAL(l(a)*p(a),0,1e-9)
	DOUBLES_EQUAL(l(b)*q(b),0,1e-9)
	DOUBLES_EQUAL(p(a)*l(a),0,1e-9)
	DOUBLES_EQUAL(q(a)*l(a),0,1e-9)
}

/* ************************************************************************* */
TEST( Tensor1, Incidence3D)
{
	// similar constructs exist for 3D points and planes
	Plane3h pi = plane3h(0, 1, 0, -2);
	Point3h P = point3h(0, 2, 0, 1), Q = point3h(1, 2, 0, 1);

	// Incidence is checked similarly
	DOUBLES_EQUAL(pi(A)*P(A),0,1e-9)
	DOUBLES_EQUAL(pi(A)*Q(A),0,1e-9)
	DOUBLES_EQUAL(P(A)*pi(A),0,1e-9)
	DOUBLES_EQUAL(Q(A)*pi(A),0,1e-9)
}

/* ************************************************************************* */
// Tensor2
/* ************************************************************************* */
TEST( Tensor2, Outer33)
{
	Line2h l1 = line2h(1, 2, 3), l2 = line2h(1, 3, 5);

	// We can also create tensors directly from data
	double data[3][3] = { { 1, 2, 3 }, { 3, 6, 9 }, {5, 10, 15} };
	Tensor2<3, 3> expected(data);
	// in this case expected(0) == {1,2,3}
	Line2h l0 = expected(a,b)(0);
	CHECK(l0(a) == l1(a))

	// And we create rank 2 tensors from the outer product of two rank 1 tensors
	CHECK(expected(a,b) == l1(a) * l2(b))

	// swap just swaps how you access a tensor, but note the data is the same
	CHECK(assert_equality(expected(a,b).swap(), l2(b) * l1(a)));
}

/* ************************************************************************* */
TEST( Tensor2, AnotherOuter33)
{
	// first cube point from testFundamental, projected in left and right
//	Point2h p = point2h(0, -1, 2), q = point2h(-2, -1, 2);
//	print(p(a)*q(b));
//	print(p(b)*q(a));
//	print(q(a)*p(b));
//	print(q(b)*p(a));
}

/* ************************************************************************* */
TEST( Tensor2, Outer34)
{
	Line2h l = line2h(1, 2, 3);
	Plane3h pi = plane3h(1, 3, 5, 7);
	double
			data[4][3] = { { 1, 2, 3 }, { 3, 6, 9 }, { 5, 10, 15 }, { 7, 14, 21 } };
	Tensor2<3, 4> expected(data);
	CHECK(assert_equality(expected(a,B),l(a) * pi(B)))
	CHECK(assert_equality(expected(a,B).swap(),pi(B) * l(a)))
}

/* ************************************************************************* */
TEST( Tensor2, SpecialContract)
{
	double data[3][3] = { { 1, 2, 3 }, { 2, 4, 6 }, { 3, 6, 9 } };
	Tensor2<3, 3> S(data), T(data);
	//print(S(a, b) * T(a, c)); // contract a -> b,c
	// S(a,0)*T(a,0) = [1 2 3] . [1 2 3] = 14
	// S(a,0)*T(a,2) = [1 2 3] . [3 6 9] = 3+12+27 = 42
	double data2[3][3] = { { 14, 28, 42 }, { 28, 56, 84 }, { 42, 84, 126 } };
	Tensor2<3, 3> expected(data2);
	CHECK(assert_equality(expected(b,c), S(a, b) * T(a, c)));
}

/* ************************************************************************* */
TEST( Tensor2, ProjectiveCamera)
{
	Point2h p = point2h(1 + 2, 2, 5);
	Point3h P = point3h(1, 2, 5, 1);
	double data[4][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 2, 0, 0 } };
	ProjectiveCamera M(data);
	CHECK(assert_equality(p(a),M(a,A)*P(A)))
}

/* ************************************************************************* */
namespace camera {
	// to specify the tensor M(a,A), we need to give four 2D points
	double data[4][3] = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 }, { 10, 11, 12 } };
	ProjectiveCamera M(data);
	Matrix matrix = Matrix_(4,3,1.,2.,3.,4.,5.,6.,7.,8.,9.,10.,11.,12.);
	Vector vector = Vector_( 12,1.,2.,3.,4.,5.,6.,7.,8.,9.,10.,11.,12.);
}

/* ************************************************************************* */
TEST( Tensor2, reshape )
{
	// it is annoying that a camera can only be reshaped to a 4*3
//	print(camera::M(a,A));
	Matrix actual = reshape(camera::M(a,A),4,3);
	EQUALITY(camera::matrix,actual);
}

/* ************************************************************************* */
TEST( Tensor2, toVector )
{
	// Vectors are created with the leftmost indices iterating the fastest
	Vector actual = toVector(camera::M(a,A));
	CHECK(assert_equal(camera::vector,actual));
}

/* ************************************************************************* */
TEST( Tensor2, reshape2 )
{
	Tensor2<3,4> actual = reshape2<3,4>(camera::vector);
	CHECK(assert_equality(camera::M(a,A),actual(a,A)));
}

/* ************************************************************************* */
TEST( Tensor2, reshape_33_to_9 )
{
	double data[3][3] = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
	FundamentalMatrix F(data);
	Matrix matrix = Matrix_(1,9,1.,2.,3.,4.,5.,6.,7.,8.,9.);
	Matrix actual = reshape(F(a,b),1,9);
	EQUALITY(matrix,actual);
	Vector v = Vector_( 9,1.,2.,3.,4.,5.,6.,7.,8.,9.);
	CHECK(assert_equality(F(a,b),reshape2<3, 3> (v)(a,b)));
}

/* ************************************************************************* */
// Tensor3
/* ************************************************************************* */
TEST( Tensor3, Join)
{
	Line2h l = line2h(-13, 5, 1);
	Point2h p = point2h(1, 2, 3), q = point2h(2, 5, 1);

	// join points into line
	Eta3 e;
	CHECK(assert_equality(e(a, b, c) * p(a) * q(b), l(c)))
}

/* ************************************************************************* */
TEST( Tensor5, Outer32)
{
	double t[3][3][3] = { { { 0, 0, 3 }, { 0, 8, -125 }, { -3, 125, 1 } }, { { 0,
			0, 3 }, { 0, 8, -125 }, { -3, 125, 1 } }, { { 0, 0, 3 }, { 0, 8, -125 },
			{ -3, 125, 1 } } };
	TrifocalTensor T(t);

	double data[3][3] = { { 0, 0, 3 }, { 0, 8, -125 }, { -3, 125, 1 } };
	FundamentalMatrix F(data);

	//Index<3, 'd'> d, _d;
	//Index<3, 'e'> e, _e;
	//print(T(_a,b,c)*F(_d,_e));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

