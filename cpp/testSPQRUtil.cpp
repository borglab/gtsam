/**
 * @file   testSPQRUtil.cpp
 * @brief  Unit test for SPQR utility functions
 * @author Kai Ni
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include "SPQRUtil.h"

using namespace std;
using namespace gtsam;

#ifdef GT_USE_LAPACK
/* ************************************************************************* */
TEST(SPQRUtil, MakeStair)
{
	double data[] = { -5, 0, 5, 0, 0, 0, -1,
										00,-5, 0, 5, 0, 0, 1.5,
										10, 0, 0,	0,-10,0,   2,
										00, 10,0, 0, 0, -10, -1 };
	Matrix A = Matrix_(4, 7, data);

	long* Stair = MakeStairs(A);

	double data2[] = { -5, 0, 5, 0, 0, 0, -1,
										10, 0, 0,	0,-10,0,   2,
										00,-5, 0, 5, 0, 0, 1.5,
										00, 10,0, 0, 0, -10, -1 };
	Matrix A_expected = Matrix_(4, 7, data2);
	CHECK(assert_equal(A_expected, A, 1e-10));

	long Stair_expected[] = {2, 4, 4, 4, 4, 4, 4};
	for (int i=0; i<7; i++)
		DOUBLES_EQUAL(Stair_expected[i], Stair[i], 1e-7);
	delete []Stair;
}

/* ************************************************************************* */
TEST(SPQRUtil, MakeStair2)
{
	double data[] = { 0.1, 0,	   0,	  0,
										0,	 0.3,	 0,	  0,
										0,	 0,	   0.3,	0,
										1.6,-0.2,	-2.5,	0.2,
										0,	 1.6,	 0.7,	0.1,
										0,	 0,	  -7.8,	0.7 };
	Matrix A = Matrix_(6, 4, data);

	long* Stair = MakeStairs(A);

	double data2[] = { 0.1, 0,	   0,	  0,
										 1.6,-0.2,	-2.5,	0.2,
									 	 0,	 0.3,	 0,	  0,
										 0,	 1.6,	 0.7,	0.1,
										 0,	 0,	   0.3,	0,
										 0,	 0,	  -7.8,	0.7
 };
	Matrix A_expected = Matrix_(6, 4, data2);
	CHECK(assert_equal(A_expected, A, 1e-10));

	long Stair_expected[] = {2, 4, 6, 6};
	for (int i=0; i<4; i++)
		DOUBLES_EQUAL(Stair_expected[i], Stair[i], 1e-7);
	delete []Stair;
}

/* ************************************************************************* */
TEST(SPQRUtil, houseHolder_spqr)
{
	double data[] = { -5, 0, 5, 0, 0, 0, -1,
										00,-5, 0, 5, 0, 0, 1.5,
										10, 0, 0,	0,-10,0,   2,
										00, 10,0, 0, 0, -10, -1 };

	// check in-place householder, with v vectors below diagonal
	double data1[] = { 11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236,
										0, 11.1803,	0, -2.2361, 0, -8.9443, -1.565,
										0, 0, 4.4721, 0, -4.4721,	0, 0,
										0, 0, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected1 = Matrix_(4, 7, data1);
	Matrix A1 = Matrix_(4, 7, data);
	householder_spqr(A1);
	CHECK(assert_equal(expected1, A1, 1e-3));
}

/* ************************************************************************* */
TEST(SPQRUtil, houseHolder_spqr2)
{
	double data[] = { -5, 0, 5, 0, 0, 0, -1,
										00,-5, 0, 5, 0, 0, 1.5,
										10, 0, 0,	0,-10,0,   2,
										00, 10,0, 0, 0, -10, -1 };

	// check in-place householder, with v vectors below diagonal
	double data1[] = { 11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236,
										0, -11.1803,	0, 2.2361, 0, 8.9443, 1.565,
										0, 0, -4.4721, 0, 4.4721,	0, 0,
										0, 0, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected1 = Matrix_(4, 7, data1);
	Matrix A1 = Matrix_(4, 7, data);
	long* Stair = MakeStairs(A1);
	householder_spqr(A1, Stair);
	CHECK(assert_equal(expected1, A1, 1e-3));
}

/* ************************************************************************* */
TEST(SPQRUtil, houseHolder_spqr3)
{
	double data[] = { 1, 1, 9,
										1, 0, 5};

	// check in-place householder, with v vectors below diagonal
	double data1[] = {-sqrt(2),	-1/sqrt(2), -7*sqrt(2),
			 	 	 	 	 	 	 	 0,	-1/sqrt(2), -4/sqrt(2)};
	Matrix expected1 = Matrix_(2, 3, data1);
	Matrix A1 = Matrix_(2, 3, data);
	householder_spqr(A1);
	CHECK(assert_equal(expected1, A1, 1e-3));
}
#endif

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
