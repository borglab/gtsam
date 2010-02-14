/*
 * testHomography2.cpp
 * @brief Test and estimate 2D homographies
 * Created on: Feb 13, 2010
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

/* ************************************************************************* */
TEST( Homography2, RealImages)
{
	// 4 point correspondences MATLAB from the floor of bt001.png and bt002.png
	Correspondence p1(point2h(216.841, 443.220, 1), point2h(213.528, 414.671, 1));
	Correspondence p2(point2h(252.119, 363.481, 1), point2h(244.614, 348.842, 1));
	Correspondence p3(point2h(316.614, 414.768, 1), point2h(303.128, 390.000, 1));
	Correspondence p4(point2h(324.165, 465.463, 1), point2h(308.614, 431.129, 1));

	// Homography obtained using MATLAB code
	double h[3][3] = { { 0.9075, 0.0031, -0 }, { -0.1165, 0.8288, -0.0004 }, {
			30.8472, 16.0449, 1 } };
	Homography2 H(h);

	// CHECK whether they are equivalent
	CHECK(assert_equivalent(p1.second(b),H(b,a)*p1.first(a),0.05))
	CHECK(assert_equivalent(p2.second(b),H(b,a)*p2.first(a),0.05))
	CHECK(assert_equivalent(p3.second(b),H(b,a)*p3.first(a),0.05))
	CHECK(assert_equivalent(p4.second(b),H(b,a)*p4.first(a),0.05))
}

/* ************************************************************************* */
// Homography test case
// 4 trivial correspondences of a translating square
Correspondence p1(point2h(0, 0, 1), point2h(4, 5, 1));
Correspondence p2(point2h(1, 0, 1), point2h(5, 5, 1));
Correspondence p3(point2h(1, 1, 1), point2h(5, 6, 1));
Correspondence p4(point2h(0, 1, 1), point2h(4, 6, 1));

double h[3][3] = { { 1, 0, 4 }, { 0, 1, 5 }, { 0, 0, 1 } };
Homography2 H(h);

/* ************************************************************************* */
TEST( Homography2, TestCase)
{
	// Check homography
	list<Correspondence> correspondences;
	correspondences += p1, p2, p3, p4;
	BOOST_FOREACH(const Correspondence& p, correspondences)
		CHECK(assert_equality(p.second(b),H(_a,b) * p.first(a)))

	// Check a line
	Line2h l1 = line2h(1, 0, -1); // in a
	Line2h l2 = line2h(1, 0, -5); // x==5 in b
	CHECK(assert_equality(l1(a), H(a,b)*l2(b)))
}

/* ************************************************************************* */
TEST( Homography2, Estimate)
{
	list<Correspondence> correspondences;
	correspondences += p1, p2, p3, p4;
	Homography2 estimatedH = estimateHomography2(correspondences);
	CHECK(assert_equivalent(H(_a,b),estimatedH(_a,b)));
}

/* ************************************************************************* */
TEST( Homography2, EstimateReverse)
{
	double h[3][3] = { { 1, 0, -4 }, { 0, 1, -5 }, { 0, 0, 1 } };
	Homography2 reverse(h);

	list<Correspondence> correspondences;
	correspondences += p1.swap(), p2.swap(), p3.swap(), p4.swap();
	Homography2 estimatedH = estimateHomography2(correspondences);
	CHECK(assert_equality(reverse(_a,b),estimatedH(_a,b)*(1.0/estimatedH(2,2))));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

