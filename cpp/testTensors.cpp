/*
 * testTensors.cpp
 * @brief try tensor expressions based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 9, 2010
 * @author: Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "tensors.h"
#include "tensorInterface.h"
#include "projectiveGeometry.h"

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
	Point2h p = point2h(1, 2, 3), q = point2h(2, 4, 6);
	CHECK(p(a)==p(a))
	CHECK(assert_equality(p(a),p(a)))
	CHECK(assert_equivalent(p(a),q(a)))
	DOUBLES_EQUAL(sqrt(14),norm(p(a)),1e-9)
	CHECK(assert_equality(p(a)*2,q(a)))
}

/* ************************************************************************* */
TEST( Tensor1, Incidence2D)
{
	Line2h l = line2h(-13, 5, 1);
	Point2h p = point2h(1, 2, 3), q = point2h(2, 5, 1);

	// incidence
	DOUBLES_EQUAL(l(a)*p(a),0,1e-9)
	DOUBLES_EQUAL(l(a)*q(a),0,1e-9)
	DOUBLES_EQUAL(p(a)*l(a),0,1e-9)
	DOUBLES_EQUAL(q(a)*l(a),0,1e-9)
}

/* ************************************************************************* */
TEST( Tensor1, Incidence3D)
{
	Plane3h pi = plane3h(0, 1, 0, -2);
	Point3h P = point3h(0, 2, 0, 1), Q = point3h(1, 2, 0, 1);

	// incidence
	DOUBLES_EQUAL(pi(A)*P(A),0,1e-9)
	DOUBLES_EQUAL(pi(A)*Q(A),0,1e-9)
	DOUBLES_EQUAL(P(A)*pi(A),0,1e-9)
	DOUBLES_EQUAL(Q(A)*pi(A),0,1e-9)
}

/* ************************************************************************* */
// Tensor2
/* ************************************************************************* */
TEST( Tensor2, Outer3)
{
	Line2h l1 = line2h(1, 2, 3), l2 = line2h(1, 3, 5);

	double data[3][3] = { { 1, 2, 3 }, { 2, 4, 6 }, { 3, 6, 9 } };
	Tensor2<3, 3> expected(data);
	CHECK(expected(a,b) == expected(a,b))
	CHECK(expected(a,b) == l1(a) * l1(b))
	CHECK(expected(a,b).swap() == l1(b) * l1(a))
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

	Index<3, 'd'> d, _d;
	Index<3, 'e'> e, _e;
	//print(T(_a,b,c)*F(_d,_e));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

