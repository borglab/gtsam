/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testFundamental.cpp
 * @brief try tensor expressions based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 13, 2010
 * @author: Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/geometry/tensors.h>
#include <gtsam/geometry/tensorInterface.h>
#include <gtsam/geometry/projectiveGeometry.h>

using namespace std;
using namespace gtsam;
using namespace tensors;

/* ************************************************************************* */
// Indices

Index<3, 'a'> a;
Index<3, 'b'> b;

Index<4, 'A'> A;
Index<4, 'B'> B;

/* ************************************************************************* */
TEST( Tensors, FundamentalMatrix)
{
	double f[3][3] = { { 1, 0, 0 }, { 1, 2, 3 }, { 1, 2, 3 } };
	FundamentalMatrix F(f);

	Point2h p = point2h(1, 2, 3); // point p in view one
	Point2h q = point2h(14, -1, 0); // point q in view two

	// points p and q are in correspondence
	CHECK(F(a,b)*p(a)*q(b) == 0)

	// in detail, l1(b)*q(b)==0
	Line2h l1 = line2h(1, 14, 14);
	CHECK(F(a,b)*p(a) == l1(b))
	CHECK(l1(b)*q(b) == 0); // q is on line l1

	// and l2(a)*p(a)==0
	Line2h l2 = line2h(13, -2, -3);
	CHECK(F(a,b)*q(b) == l2(a))
	CHECK(l2(a)*p(a) == 0); // p is on line l2
}

/* ************************************************************************* */
// Stereo setup, -1,1
/* ************************************************************************* */
// t points towards origin
double left__[4][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { +1, 0, 0 } };
double right_[4][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { -1, 0, 0 } };
//double right_[4][3] = { { cos(0.1), -sin(0.1), 0 }, { sin(0.1), cos(0.1), 0 }, { 0, 0, 1 }, { -1, 0, 0 } };
ProjectiveCamera ML(left__), MR(right_);

// Cube
Point3h P1 = point3h(-1, -1, 3 - 1, 1);
Point3h P2 = point3h(-1, -1, 3 + 1, 1);
Point3h P3 = point3h(-1, +1, 3 - 1, 1);
Point3h P4 = point3h(-1, +1, 3 + 1, 1);
Point3h P5 = point3h(+1, -1, 3 - 1, 1);
Point3h P6 = point3h(+1, -1, 3 + 1, 1);
Point3h P7 = point3h(+1, +1, 3 - 1, 1);
Point3h P8 = point3h(+1, +1, 3 + 1, 1);

/* ************************************************************************* */
TEST( Tensors, FundamentalMatrix2)
{
	// The correct fundamental matrix is given by formula 9.1 in HZ 2nd ed., p. 244
	// F = \hat(e')P'P+
	// and is very simple
	double f[3][3] = {{0,0,0}
	,{0,0,-1}
	,{0,1,0}
	};
	FundamentalMatrix F(f);

	// Create a list of correspondences
	list<Point3h> points;
	Point3h P9 = point3h(-2,3,4,1);
	Point3h P10 = point3h(1,1,5,1);
	points += P1, P2, P3, P4, P5, P6, P7, P8, P9, P10;
	list<Correspondence> correspondences;
	BOOST_FOREACH(const Point3h& P, points) {
		Correspondence p(ML(a,A)*P(A), MR(b,A)*P(A));
		DOUBLES_EQUAL(0,F(a,b) * p.first(a) * p.second(b),1e-9);
		correspondences += p;
	}

	// let's check it for another arbitrary point
	Point2h left(ML(a,A)*P9(A)), right(MR(b,A)*P9(A));
	DOUBLES_EQUAL(0,F(a,b) * left(a) * right(b),1e-9);

	FundamentalMatrix actual = estimateFundamentalMatrix(correspondences);
	CHECK(assert_equivalent(F(a,b),actual(a,b),0.1));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

