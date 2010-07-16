/*
 * testTrifocal.cpp
 * @brief trifocal tensor estimation
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
Index<3, 'd'> d, _d;
Index<3, 'e'> e, _e;
Index<3, 'f'> f, _f;
Index<3, 'g'> g, _g;

Index<4, 'A'> A;

/* ************************************************************************* */
// 3 Camera setup in trifocal stereo setup, -1,0,1
/* ************************************************************************* */
double left__[4][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { -1, 0, 0 } };
double middle[4][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { +0, 0, 0 } };
double right_[4][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { +1, 0, 0 } };
ProjectiveCamera ML(left__), MM(middle), MR(right_);

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
// Manohar's homework
TEST(Tensors, TrifocalTensor)
{
	// Checked with MATLAB !
	double t[3][3][3] = {
		{ { -0.301511, 0, 0 }, { 0, -0.603023, 0 }, { 0, 0,-0.603023 } },
		{ {  0, 0.301511, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
		{ {  0, 0, 0.301511 }, { 0, 0, 0 }, { 0, 0, 0 } }
	};
	TrifocalTensor T(t);
	//print(T(a,b,c));

	list<Point3h> points;
	points += P1, P2, P3, P4, P5, P6, P7, P8;

	Eta3 eta;

	list<Triplet> triplets;
	double data[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	Tensor2<3,3> zero(data);
	BOOST_FOREACH(const Point3h& P, points) {
		// form triplet
		Triplet p(ML(a,A)*P(A), MM(b,A)*P(A), MR(c,A)*P(A));
		// check trifocal constraint
		Tensor2<3,3> T1 = T(_a,b,c) * p.first(a);
		Tensor2<3,3> T2 = eta(_d,_b,_e) * p.second(d);
		Tensor2<3,3> T3 = eta(_f,_c,_g) * p.third(f);
		CHECK(assert_equality(zero(_e,_g), (T1(b,c) * T2(_b,_e)) * T3(_c,_g),1e-4));
		triplets += p;
	}

	// We will form the rank 5 tensor by multiplying a rank 3 and rank 2
	// Let's check the answer for the first point:
	Triplet p = triplets.front();

	// This checks the rank 3 (with answer checked in MATLAB);
	double matlab3[3][3][3] = {
		{{ -0, -0, 0}, { 4, 2, -4}, { 2, 1, -2}},
		{{ -4, -2, 4}, {-0, -0, 0}, {-2, -1, 2}},
		{{ -2, -1, 2}, { 2, 1, -2}, {-0, -0, 0}}
	};
	Tensor3<3,3,3> expected3(matlab3);
	CHECK(assert_equality(expected3(a,_b,_e), p.first(a)* (eta(_d,_b,_e) * p.second(d))));

	// This checks the rank 2 (with answer checked in MATLAB);
	double matlab2[3][3] = { {0, -2, -1}, {2, 0, 0}, {1, 0, 0}};
	Tensor2<3,3> expected2(matlab2);
	CHECK(assert_equality(expected2(_c,_g), eta(_f,_c,_g) * p.third(f)));

	TrifocalTensor actual = estimateTrifocalTensor(triplets);

	//print(actual(a,b,c));
	CHECK(assert_equality(T(_a,b,c),actual(_a,b,c),1e-6));
}

TEST(Tensors, TrifocalTensor1){
	// Manually clicked points
	// Points in frame1
	// 339   336   281    51   367   265   135
	// 152   344   246   210    76   248   246
	// Points in frame2
	// 380   381   311   108   395   294   161
	// 148   340   242   208    73   243   242
	// Points in frame3
	// 440   441   360   181   444   344   207
	// 151   343   246   212    74   247   245

	Triplet p1(point2h(339,152,1), point2h(380,148,1), point2h(440,151,1));
	Triplet p2(point2h(336,344,1), point2h(381,340,1), point2h(441,343,1));
	Triplet p3(point2h(281,246,1), point2h(311,242,1), point2h(360,246,1));
	Triplet p4(point2h(51,210,1 ), point2h(108,208,1), point2h(181,212,1));
	Triplet p5(point2h(367,76,1 ), point2h(395,73,1 ), point2h(444,74,1) );
	Triplet p6(point2h(265,248,1), point2h(294,243,1), point2h(344,247,1));
	Triplet p7(point2h(135,246,1), point2h(161,242,1), point2h(207,245,1));
	list<Triplet> triplets;
	triplets += p1, p2, p3, p4, p5, p6, p7;

	// Checked with MATLAB !
	/*double t[3][3][3] = {
	{ {	-0.0145,0.0081,0.0000}, {-0.0004,-0.0180,0.0000}, {0.2334,-0.6283,-0.0230}},
	{ {	-0.0162,-0.0001,0.0000}, {0.0049,-0.0075,0.0000}, {0.7406,0.0209,-0.0140}},
	{ {	-0.0001,-0.0000,-0.0000}, {-0.0000,-0.0001,0.0000}, {0.0096,0.0063,-0.0000}}
	};*/
	double t[3][3][3] = {
	{ {	0.0145,0.0004,-0.2334}, {-0.0081,0.0180,0.6283}, {0.0000,0.0000,0.0230}},
	{ {	0.0162,-0.0049,-0.7406}, {0.0001,-0.0075,-0.0209}, {0.0000,0.0000,0.0140}},
	{ {	0.0001,-0.0000,-0.0096}, {0.0000,0.0001,-0.0063}, {0.0000,-0.0000,-0.0000}}
	};
	TrifocalTensor T(t);

	TrifocalTensor actual = estimateTrifocalTensor(triplets);

	Eta3 eta;
	double data[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	Tensor2<3,3> zero(data);

	BOOST_FOREACH(const Triplet& t, triplets) {
		Tensor2<3,3> T1 = actual(_a,b,c) * t.first(a);
		Tensor2<3,3> T2 = eta(_d,_b,_e) * t.second(d);
		Tensor2<3,3> T3 = eta(_f,_c,_g) * t.third(f);
		CHECK(assert_equality(zero(_e,_g), (T1(b,c) * T2(_b,_e)) * T3(_c,_g),1e-2));
	}

	CHECK(assert_equality(T(_a,b,c), actual(_a,b,c),1e-1));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

