/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testHomography2.cpp
 * @brief Test and estimate 2D homographies
 * @date Feb 13, 2010
 * @author Frank Dellaert
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam2/geometry/tensors.h>
#include <gtsam2/geometry/tensorInterface.h>
#include <gtsam2/geometry/projectiveGeometry.h>
#include <gtsam/geometry/Pose3.h>

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
/**
 * Computes the homography H(I,_T) from template to image
 * given the pose tEc of the camera in the template coordinate frame.
 * Assumption is Z is normal to the template, template texture in X-Y plane.
 */
Homography2 patchH(const Pose3& tEc) {
	Pose3 cEt = tEc.inverse();
	Rot3 cRt = cEt.rotation();
	Point3 r1 = cRt.r1(), r2 = cRt.r2(), t = cEt.translation();

	// TODO cleanup !!!!
	// column 1
	double H11 = r1.x();
	double H21 = r1.y();
	double H31 = r1.z();
	// column 2
	double H12 = r2.x();
	double H22 = r2.y();
	double H32 = r2.z();
	// column 3
	double H13 = t.x();
	double H23 = t.y();
	double H33 = t.z();
	double data2[3][3] = { { H11, H21, H31 }, { H12, H22, H32 },
			{ H13, H23, H33 } };
	return Homography2(data2);
}

/* ************************************************************************* */
namespace gtsam {
//	size_t dim(const tensors::Tensor2<3, 3>& H) {return 9;}
	Vector toVector(const tensors::Tensor2<3, 3>& H) {
		Index<3, 'T'> _T; // covariant 2D template
		Index<3, 'C'> I; // contravariant 2D camera
		return toVector(H(I,_T));
	}
	Vector localCoordinates(const tensors::Tensor2<3, 3>& A, const tensors::Tensor2<3, 3>& B) {
		return toVector(A)-toVector(B); // TODO correct order ?
	}
}

#include <gtsam/base/numericalDerivative.h>

/* ************************************************************************* */
TEST( Homography2, patchH)
{
	Index<3, 'T'> _T; // covariant 2D template
	Index<3, 'C'> I; // contravariant 2D camera

	// data[_T][I]
	double data1[3][3] = {{1,0,0},{0,-1,0},{0,0,10}};
	Homography2 expected(data1);

	// camera rotation, looking in negative Z
	Rot3 gRc(Point3(1,0,0),Point3(0,-1,0),Point3(0,0,-1));
	Point3 gTc(0,0,10); // Camera location, out on the Z axis
	Pose3 gEc(gRc,gTc); // Camera pose

	Homography2 actual = patchH(gEc);

//	GTSAM_PRINT(expected(I,_T));
//	GTSAM_PRINT(actual(I,_T));
	CHECK(assert_equality(expected(I,_T),actual(I,_T)));

	// FIXME: this doesn't appear to be tested, and requires that Tensor2 be a Lie object
//  Matrix D = numericalDerivative11<Homography2,Pose3>(patchH, gEc);
//  print(D,"D");
}

/* ************************************************************************* */
TEST( Homography2, patchH2)
{
	Index<3, 'T'> _T; // covariant 2D template
	Index<3, 'C'> I; // contravariant 2D camera

	// data[_T][I]
	double data1[3][3] = {{1,0,0},{0,-1,0},{0,0,10}};
	Homography2 expected(data1);

	// camera rotation, looking in negative Z
	Rot3 gRc(Point3(1,0,0),Point3(0,-1,0),Point3(0,0,-1));
	Point3 gTc(0,0,10); // Camera location, out on the Z axis
	Pose3 gEc(gRc,gTc); // Camera pose

	Homography2 actual = patchH(gEc);

//	GTSAM_PRINT(expected(I,_T));
//	GTSAM_PRINT(actual(I,_T));
	CHECK(assert_equality(expected(I,_T),actual(I,_T)));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

