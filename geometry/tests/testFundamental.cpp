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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

