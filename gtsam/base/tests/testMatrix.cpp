/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testMatrix.cpp
 * @brief  Unit test for Matrix Library
 * @author Christian Potthast
 * @author Carlos Nieto
 **/

#include <iostream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;

static double inf = std::numeric_limits<double>::infinity();
static const double tol = 1e-9;

/* ************************************************************************* */
TEST( matrix, constructor_data )
{
	double data[] = { -5, 3, 0, -5 };
	Matrix A = Matrix_(2, 2, data);

	Matrix B(2, 2);
	B(0, 0) = -5;
	B(0, 1) = 3;
	B(1, 0) = 0;
	B(1, 1) = -5;

	EQUALITY(A,B);
}

/* ************************************************************************* */
TEST( matrix, constructor_vector )
{
	double data[] = { -5, 3, 0, -5 };
	Matrix A = Matrix_(2, 2, data);
	Vector v(4);
	copy(data, data + 4, v.data());
	Matrix B = Matrix_(2, 2, v); // this one is column order !
	EQUALITY(A,trans(B));
}

/* ************************************************************************* */
TEST( matrix, Matrix_ )
{
	Matrix A = Matrix_(2, 2, -5.0, 3.0, 00.0, -5.0);
	Matrix B(2, 2);
	B(0, 0) = -5;
	B(0, 1) = 3;
	B(1, 0) = 0;
	B(1, 1) = -5;

	EQUALITY(A,B);

}

/* ************************************************************************* */
TEST( matrix, col_major )
{
	Matrix A = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	const double * const a = &A(0, 0);
	EXPECT_DOUBLES_EQUAL(1, a[0], tol);
	EXPECT_DOUBLES_EQUAL(3, a[1], tol);
	EXPECT_DOUBLES_EQUAL(2, a[2], tol);
	EXPECT_DOUBLES_EQUAL(4, a[3], tol);
}

/* ************************************************************************* */
TEST( matrix, collect1 )
{
	Matrix A = Matrix_(2, 2, -5.0, 3.0, 00.0, -5.0);
	Matrix B = Matrix_(2, 3, -0.5, 2.1, 1.1, 3.4, 2.6, 7.1);
	Matrix AB = collect(2, &A, &B);
	Matrix C(2, 5);
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			C(i, j) = A(i, j);
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 3; j++)
			C(i, j + 2) = B(i, j);

	EQUALITY(C,AB);
}

/* ************************************************************************* */
TEST( matrix, collect2 )
{
	Matrix A = Matrix_(2, 2, -5.0, 3.0, 00.0, -5.0);
	Matrix B = Matrix_(2, 3, -0.5, 2.1, 1.1, 3.4, 2.6, 7.1);
	vector<const Matrix*> matrices;
	matrices.push_back(&A);
	matrices.push_back(&B);
	Matrix AB = collect(matrices);
	Matrix C(2, 5);
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			C(i, j) = A(i, j);
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 3; j++)
			C(i, j + 2) = B(i, j);

	EQUALITY(C,AB);
}

/* ************************************************************************* */
TEST( matrix, collect3 )
{
	Matrix A, B;
	A = eye(2, 3);
	B = eye(2, 3);
	vector<const Matrix*> matrices;
	matrices.push_back(&A);
	matrices.push_back(&B);
	Matrix AB = collect(matrices, 2, 3);
	Matrix exp = Matrix_(2, 6,
			1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0, 1.0, 0.0);

	EQUALITY(exp,AB);
}

/* ************************************************************************* */
TEST( matrix, stack )
{
	Matrix A = Matrix_(2, 2, -5.0, 3.0, 00.0, -5.0);
	Matrix B = Matrix_(3, 2, -0.5, 2.1, 1.1, 3.4, 2.6, 7.1);
	Matrix AB = stack(2, &A, &B);
	Matrix C(5, 2);
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			C(i, j) = A(i, j);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 2; j++)
			C(i + 2, j) = B(i, j);

	EQUALITY(C,AB);
}

/* ************************************************************************* */
TEST( matrix, column )
{
	Matrix A = Matrix_(4, 7, -1., 0., 1., 0., 0., 0., -0.2, 0., -1., 0., 1.,
			0., 0., 0.3, 1., 0., 0., 0., -1., 0., 0.2, 0., 1., 0., 0., 0., -1.,
			-0.1);
	Vector a1 = column(A, 0);
	Vector exp1 = Vector_(4, -1., 0., 1., 0.);
	EXPECT(assert_equal(a1, exp1));

	Vector a2 = column(A, 3);
	Vector exp2 = Vector_(4, 0., 1., 0., 0.);
	EXPECT(assert_equal(a2, exp2));

	Vector a3 = column(A, 6);
	Vector exp3 = Vector_(4, -0.2, 0.3, 0.2, -0.1);
	EXPECT(assert_equal(a3, exp3));
}

/* ************************************************************************* */
TEST( matrix, insert_column )
{
	Matrix big = zeros(5, 6);
	Vector col = ones(5);
	size_t j = 3;

	insertColumn(big, col, j);

	Matrix expected = Matrix_(5, 6,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

	EXPECT(assert_equal(expected, big));
}

/* ************************************************************************* */
TEST( matrix, insert_subcolumn )
{
	Matrix big = zeros(5, 6);
	Vector col1 = ones(2);
	size_t i = 1;
	size_t j = 3;

	insertColumn(big, col1, i, j); // check 1

	Vector col2 = ones(1);
	insertColumn(big, col2, 4, 5); // check 2

	Matrix expected = Matrix_(5, 6,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	EXPECT(assert_equal(expected, big));
}

/* ************************************************************************* */
TEST( matrix, row )
{
	Matrix A = Matrix_(4, 7, -1., 0., 1., 0., 0., 0., -0.2, 0., -1., 0., 1.,
			0., 0., 0.3, 1., 0., 0., 0., -1., 0., 0.2, 0., 1., 0., 0., 0., -1.,
			-0.1);
	Vector a1 = row(A, 0);
	Vector exp1 = Vector_(7, -1., 0., 1., 0., 0., 0., -0.2);
	EXPECT(assert_equal(a1, exp1));

	Vector a2 = row(A, 2);
	Vector exp2 = Vector_(7, 1., 0., 0., 0., -1., 0., 0.2);
	EXPECT(assert_equal(a2, exp2));

	Vector a3 = row(A, 3);
	Vector exp3 = Vector_(7, 0., 1., 0., 0., 0., -1., -0.1);
	EXPECT(assert_equal(a3, exp3));
}

/* ************************************************************************* */
TEST( matrix, zeros )
{
	Matrix A(2, 3);
	A(0, 0) = 0;
	A(0, 1) = 0;
	A(0, 2) = 0;
	A(1, 0) = 0;
	A(1, 1) = 0;
	A(1, 2) = 0;

	Matrix zero = zeros(2, 3);

	EQUALITY(A , zero);
}

/* ************************************************************************* */
TEST( matrix, insert_sub )
{
	Matrix big = zeros(5, 6), small = Matrix_(2, 3, 1.0, 1.0, 1.0, 1.0, 1.0,
			1.0);

	insertSub(big, small, 1, 2);

	Matrix expected = Matrix_(5, 6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	EXPECT(assert_equal(expected, big));
}

/* ************************************************************************* */
TEST( matrix, stream_read ) {
  Matrix expected = Matrix_(3,4,
    1.1, 2.3, 4.2, 7.6,
    -0.3, -8e-2, 5.1, 9.0,
    1.2, 3.4, 4.5, 6.7);

  string matrixAsString = 
    "1.1 2.3 4.2 7.6\n"
    "-0.3 -8e-2 5.1    9.0\n\r" // Test extra spaces and windows newlines
    "1.2 \t 3.4 4.5 6.7"; // Test tab as separator

  stringstream asStream(matrixAsString, ios::in);

  Matrix actual;
  asStream >> actual;

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( matrix, scale_columns )
{
	Matrix A(3, 4);
	A(0, 0) = 1.;
	A(0, 1) = 1.;
	A(0, 2) = 1.;
	A(0, 3) = 1.;
	A(1, 0) = 1.;
	A(1, 1) = 1.;
	A(1, 2) = 1.;
	A(1, 3) = 1.;
	A(2, 0) = 1.;
	A(2, 1) = 1.;
	A(2, 2) = 1.;
	A(2, 3) = 1.;

	Vector v = Vector_(4, 2., 3., 4., 5.);

	Matrix actual = vector_scale(A, v);

	Matrix expected(3, 4);
	expected(0, 0) = 2.;
	expected(0, 1) = 3.;
	expected(0, 2) = 4.;
	expected(0, 3) = 5.;
	expected(1, 0) = 2.;
	expected(1, 1) = 3.;
	expected(1, 2) = 4.;
	expected(1, 3) = 5.;
	expected(2, 0) = 2.;
	expected(2, 1) = 3.;
	expected(2, 2) = 4.;
	expected(2, 3) = 5.;

	EXPECT(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( matrix, scale_rows )
{
	Matrix A(3, 4);
	A(0, 0) = 1.;
	A(0, 1) = 1.;
	A(0, 2) = 1.;
	A(0, 3) = 1.;
	A(1, 0) = 1.;
	A(1, 1) = 1.;
	A(1, 2) = 1.;
	A(1, 3) = 1.;
	A(2, 0) = 1.;
	A(2, 1) = 1.;
	A(2, 2) = 1.;
	A(2, 3) = 1.;

	Vector v = Vector_(3, 2., 3., 4.);

	Matrix actual = vector_scale(v, A);

	Matrix expected(3, 4);
	expected(0, 0) = 2.;
	expected(0, 1) = 2.;
	expected(0, 2) = 2.;
	expected(0, 3) = 2.;
	expected(1, 0) = 3.;
	expected(1, 1) = 3.;
	expected(1, 2) = 3.;
	expected(1, 3) = 3.;
	expected(2, 0) = 4.;
	expected(2, 1) = 4.;
	expected(2, 2) = 4.;
	expected(2, 3) = 4.;

	EXPECT(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( matrix, scale_rows_mask )
{
	Matrix A(3, 4);
	A(0, 0) = 1.;
	A(0, 1) = 1.;
	A(0, 2) = 1.;
	A(0, 3) = 1.;
	A(1, 0) = 1.;
	A(1, 1) = 1.;
	A(1, 2) = 1.;
	A(1, 3) = 1.;
	A(2, 0) = 1.;
	A(2, 1) = 1.;
	A(2, 2) = 1.;
	A(2, 3) = 1.;

	Vector v = Vector_(3, 2., std::numeric_limits<double>::infinity(), 4.);

	Matrix actual = vector_scale(v, A, true);

	Matrix expected(3, 4);
	expected(0, 0) = 2.;
	expected(0, 1) = 2.;
	expected(0, 2) = 2.;
	expected(0, 3) = 2.;
	expected(1, 0) = 1.;
	expected(1, 1) = 1.;
	expected(1, 2) = 1.;
	expected(1, 3) = 1.;
	expected(2, 0) = 4.;
	expected(2, 1) = 4.;
	expected(2, 2) = 4.;
	expected(2, 3) = 4.;

	EXPECT(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( matrix, equal )
{
	Matrix A(4, 4);
	A(0, 0) = -1;
	A(0, 1) = 1;
	A(0, 2) = 2;
	A(0, 3) = 3;
	A(1, 0) = 1;
	A(1, 1) = -3;
	A(1, 2) = 1;
	A(1, 3) = 3;
	A(2, 0) = 1;
	A(2, 1) = 2;
	A(2, 2) = -1;
	A(2, 3) = 4;
	A(3, 0) = 2;
	A(3, 1) = 1;
	A(3, 2) = 2;
	A(3, 3) = -2;

	Matrix A2(A);

	Matrix A3(A);
	A3(3, 3) = -2.1;

	EXPECT(A==A2);
	EXPECT(A!=A3);
}

/* ************************************************************************* */
TEST( matrix, equal_nan )
{
	Matrix A(4, 4);
	A(0, 0) = -1;
	A(0, 1) = 1;
	A(0, 2) = 2;
	A(0, 3) = 3;
	A(1, 0) = 1;
	A(1, 1) = -3;
	A(1, 2) = 1;
	A(1, 3) = 3;
	A(2, 0) = 1;
	A(2, 1) = 2;
	A(2, 2) = -1;
	A(2, 3) = 4;
	A(3, 0) = 2;
	A(3, 1) = 1;
	A(3, 2) = 2;
	A(3, 3) = -2;

	Matrix A2(A);

	Matrix A3(A);
	A3(3, 3) = inf;

	EXPECT(A!=A3);
}

/* ************************************************************************* */
TEST( matrix, addition )
{
	Matrix A = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	Matrix B = Matrix_(2, 2, 4.0, 3.0, 2.0, 1.0);
	Matrix C = Matrix_(2, 2, 5.0, 5.0, 5.0, 5.0);
	EQUALITY(A+B,C);
}

/* ************************************************************************* */
TEST( matrix, addition_in_place )
{
	Matrix A = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	Matrix B = Matrix_(2, 2, 4.0, 3.0, 2.0, 1.0);
	Matrix C = Matrix_(2, 2, 5.0, 5.0, 5.0, 5.0);
	A += B;
	EQUALITY(A,C);
}

/* ************************************************************************* */
TEST( matrix, subtraction )
{
	Matrix A = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	Matrix B = Matrix_(2, 2, 4.0, 3.0, 2.0, 1.0);
	Matrix C = Matrix_(2, 2, -3.0, -1.0, 1.0, 3.0);
	EQUALITY(A-B,C);
}

/* ************************************************************************* */
TEST( matrix, subtraction_in_place )
{
	Matrix A = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	Matrix B = Matrix_(2, 2, 4.0, 3.0, 2.0, 1.0);
	Matrix C = Matrix_(2, 2, -3.0, -1.0, 1.0, 3.0);
	A -= B;
	EQUALITY(A,C);
}

/* ************************************************************************* */
TEST( matrix, multiplication )
{
	Matrix A(2, 2);
	A(0, 0) = -1;
	A(1, 0) = 1;
	A(0, 1) = 1;
	A(1, 1) = -3;

	Matrix B(2, 1);
	B(0, 0) = 1.2;
	B(1, 0) = 3.4;

	Matrix AB(2, 1);
	AB(0, 0) = 2.2;
	AB(1, 0) = -9.;

	EQUALITY(A*B,AB);
}

/* ************************************************************************* */
TEST( matrix, scalar_matrix_multiplication )
{
	Vector result(2);

	Matrix A(2, 2);
	A(0, 0) = -1;
	A(1, 0) = 1;
	A(0, 1) = 1;
	A(1, 1) = -3;

	Matrix B(2, 2);
	B(0, 0) = -10;
	B(1, 0) = 10;
	B(0, 1) = 10;
	B(1, 1) = -30;

	EQUALITY((10*A),B);
}

/* ************************************************************************* */
TEST( matrix, matrix_vector_multiplication )
{
	Vector result(2);

	Matrix A = Matrix_(2, 3, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
	Vector v = Vector_(3, 1., 2., 3.);
	Vector Av = Vector_(2, 14., 32.);
	Vector AtAv = Vector_(3, 142., 188., 234.);

	EQUALITY(A*v,Av);
	EQUALITY(A^Av,AtAv);
}

/* ************************************************************************* */
TEST( matrix, nrRowsAndnrCols )
{
	Matrix A(3, 6);
	LONGS_EQUAL( A.rows() , 3 );
	LONGS_EQUAL( A.cols() , 6 );
}

/* ************************************************************************* */
TEST( matrix, scalar_divide )
{
	Matrix A(2, 2);
	A(0, 0) = 10;
	A(1, 0) = 30;
	A(0, 1) = 20;
	A(1, 1) = 40;

	Matrix B(2, 2);
	B(0, 0) = 1;
	B(1, 0) = 3;
	B(0, 1) = 2;
	B(1, 1) = 4;

	EQUALITY(B,A/10);
}

/* ************************************************************************* */
TEST( matrix, zero_below_diagonal ) {
	Matrix A1 = Matrix_(3, 4,
			1.0, 2.0, 3.0, 4.0,
	    1.0, 2.0, 3.0, 4.0,
	    1.0, 2.0, 3.0, 4.0);

	Matrix expected1 = Matrix_(3, 4,
			1.0, 2.0, 3.0, 4.0,
			0.0, 2.0, 3.0, 4.0,
			0.0, 0.0, 3.0, 4.0);
	Matrix actual1r = A1;
	zeroBelowDiagonal(actual1r);
	EXPECT(assert_equal(expected1, actual1r, 1e-10));

	Matrix actual1c = A1;
	zeroBelowDiagonal(actual1c);
	EXPECT(assert_equal(Matrix(expected1), actual1c, 1e-10));

	actual1c = A1;
	zeroBelowDiagonal(actual1c, 4);
	EXPECT(assert_equal(Matrix(expected1), actual1c, 1e-10));

	Matrix A2 = Matrix_(5, 3,
				1.0, 2.0, 3.0,
		    1.0, 2.0, 3.0,
		    1.0, 2.0, 3.0,
		    1.0, 2.0, 3.0,
		    1.0, 2.0, 3.0);
	Matrix expected2 = Matrix_(5, 3,
			1.0, 2.0, 3.0,
			0.0, 2.0, 3.0,
			0.0, 0.0, 3.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0);

	Matrix actual2r = A2;
	zeroBelowDiagonal(actual2r);
	EXPECT(assert_equal(expected2, actual2r, 1e-10));

	Matrix actual2c = A2;
	zeroBelowDiagonal(actual2c);
	EXPECT(assert_equal(Matrix(expected2), actual2c, 1e-10));

	Matrix expected2_partial = Matrix_(5, 3,
				1.0, 2.0, 3.0,
				0.0, 2.0, 3.0,
				0.0, 2.0, 3.0,
				0.0, 2.0, 3.0,
				0.0, 2.0, 3.0);
	actual2c = A2;
	zeroBelowDiagonal(actual2c, 1);
	EXPECT(assert_equal(Matrix(expected2_partial), actual2c, 1e-10));
}

/* ************************************************************************* */
TEST( matrix, inverse )
{
	Matrix A(3, 3);
	A(0, 0) = 1;
	A(0, 1) = 2;
	A(0, 2) = 3;
	A(1, 0) = 0;
	A(1, 1) = 4;
	A(1, 2) = 5;
	A(2, 0) = 1;
	A(2, 1) = 0;
	A(2, 2) = 6;

	Matrix Ainv = inverse(A);
	EXPECT(assert_equal(eye(3), A*Ainv));
	EXPECT(assert_equal(eye(3), Ainv*A));

	Matrix expected(3, 3);
	expected(0, 0) = 1.0909;
	expected(0, 1) = -0.5454;
	expected(0, 2) = -0.0909;
	expected(1, 0) = 0.2272;
	expected(1, 1) = 0.1363;
	expected(1, 2) = -0.2272;
	expected(2, 0) = -0.1818;
	expected(2, 1) = 0.0909;
	expected(2, 2) = 0.1818;

	EXPECT(assert_equal(expected, Ainv, 1e-4));

	// These two matrices failed before version 2003 because we called LU incorrectly
	Matrix lMg(Matrix_(3, 3, 0.0, 1.0, -2.0, -1.0, 0.0, 1.0, 0.0, 0.0, 1.0));
	EXPECT(assert_equal(Matrix_(3,3,
			0.0, -1.0, 1.0,
			1.0, 0.0, 2.0,
			0.0, 0.0, 1.0),
			inverse(lMg)));
	Matrix gMl(Matrix_(3, 3, 0.0, -1.0, 1.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0));
	EXPECT(assert_equal(Matrix_(3,3,
			0.0, 1.0,-2.0,
			-1.0, 0.0, 1.0,
			0.0, 0.0, 1.0),
			inverse(gMl)));
}

/* ************************************************************************* */
TEST( matrix, inverse2 )
{
	Matrix A(3, 3);
	A(0, 0) = 0;
	A(0, 1) = -1;
	A(0, 2) = 1;
	A(1, 0) = 1;
	A(1, 1) = 0;
	A(1, 2) = 2;
	A(2, 0) = 0;
	A(2, 1) = 0;
	A(2, 2) = 1;

	Matrix Ainv = inverse(A);

	Matrix expected(3, 3);
	expected(0, 0) = 0;
	expected(0, 1) = 1;
	expected(0, 2) = -2;
	expected(1, 0) = -1;
	expected(1, 1) = 0;
	expected(1, 2) = 1;
	expected(2, 0) = 0;
	expected(2, 1) = 0;
	expected(2, 2) = 1;

	EXPECT(assert_equal(expected, Ainv, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, backsubtitution )
{
	// TEST ONE  2x2 matrix U1*x=b1
	Vector expected1 = Vector_(2, 3.6250, -0.75);
	Matrix U22 = Matrix_(2, 2, 2., 3., 0., 4.);
	Vector b1 = U22 * expected1;
	EXPECT( assert_equal(expected1 , backSubstituteUpper(U22, b1), 0.000001));

	// TEST TWO  3x3 matrix U2*x=b2
	Vector expected2 = Vector_(3, 5.5, -8.5, 5.);
	Matrix U33 = Matrix_(3, 3, 3., 5., 6., 0., 2., 3., 0., 0., 1.);
	Vector b2 = U33 * expected2;
	EXPECT( assert_equal(expected2 , backSubstituteUpper(U33, b2), 0.000001));

	// TEST THREE  Lower triangular 3x3 matrix L3*x=b3
	Vector expected3 = Vector_(3, 1., 1., 1.);
	Matrix L3 = trans(U33);
	Vector b3 = L3 * expected3;
	EXPECT( assert_equal(expected3 , backSubstituteLower(L3, b3), 0.000001));

	// TEST FOUR Try the above with transpose backSubstituteUpper
	EXPECT( assert_equal(expected3 , backSubstituteUpper(b3,U33), 0.000001));
}

/* ************************************************************************* */
TEST( matrix, householder )
{
	double data[] = { -5, 0, 5, 0, 0, 0, -1,
			00,-5, 0, 5, 0, 0, 1.5,
			10, 0, 0,	0,-10,0,   2,
			00, 10,0, 0, 0, -10, -1 };

	// check in-place householder, with v vectors below diagonal
	double data1[] = {
			11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236,
			0, 11.1803,	0, -2.2361, 0, -8.9443, -1.565,
			-0.618034, 0, 4.4721, 0, -4.4721,	0, 0,
			0, -0.618034, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected1 = Matrix_(4, 7, data1);
	Matrix A1 = Matrix_(4, 7, data);
	householder_(A1, 3);
	EXPECT(assert_equal(expected1, A1, 1e-3));

	// in-place, with zeros below diagonal
	double data2[] = {
			11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236, 0, 11.1803,
			0, -2.2361, 0, -8.9443, -1.565, 0, 0, 4.4721, 0, -4.4721, 0, 0, 0,
			0, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected = Matrix_(4, 7, data2);
	Matrix A2 = Matrix_(4, 7, data);
	householder(A2, 3);
	EXPECT(assert_equal(expected, A2, 1e-3));
}

/* ************************************************************************* */
TEST( matrix, householder_colMajor )
{
	double data[] = {
			-5, 0, 5, 0, 0, 0, -1,
			00,-5, 0, 5, 0, 0, 1.5,
			10, 0, 0,	0,-10,0,   2,
			00, 10,0, 0, 0, -10, -1 };

	// check in-place householder, with v vectors below diagonal
	double data1[] = {
			11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236,
			0, 11.1803,	0, -2.2361, 0, -8.9443, -1.565,
			-0.618034, 0, 4.4721, 0, -4.4721,	0, 0,
			0, -0.618034, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected1(Matrix_(4, 7, data1));
	Matrix A1(Matrix_(4, 7, data));
	householder_(A1, 3);
	EXPECT(assert_equal(expected1, A1, 1e-3));

	// in-place, with zeros below diagonal
	double data2[] = {
			11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236, 0, 11.1803,
			0, -2.2361, 0, -8.9443, -1.565, 0, 0, 4.4721, 0, -4.4721, 0, 0, 0,
			0, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected(Matrix_(4, 7, data2));
	Matrix A2(Matrix_(4, 7, data));
	householder(A2, 3);
	EXPECT(assert_equal(expected, A2, 1e-3));
}

/* ************************************************************************* */
TEST( matrix, eigen_QR )
{
	// use standard Eigen function to yield a non-in-place QR factorization
	double data[] = {
			-5, 0, 5, 0, 0, 0, -1,
			00,-5, 0, 5, 0, 0, 1.5,
			10, 0, 0,	0,-10,0,   2,
			00, 10,0, 0, 0, -10, -1 };

	// in-place, with zeros below diagonal
	double data2[] = {
			11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236, 0, 11.1803,
			0, -2.2361, 0, -8.9443, -1.565, 0, 0, 4.4721, 0, -4.4721, 0, 0, 0,
			0, 0, 4.4721, 0, -4.4721, 0.894 };
	Matrix expected(Matrix_(4, 7, data2));
	Matrix A(Matrix_(4, 7, data));
	Matrix actual = A.householderQr().matrixQR();
	zeroBelowDiagonal(actual);

	EXPECT(assert_equal(expected, actual, 1e-3));

	// use shiny new in place QR inside gtsam
	A = Matrix(Matrix_(4, 7, data));
	inplace_QR(A);
	EXPECT(assert_equal(expected, A, 1e-3));
}

/* ************************************************************************* */
// unit test for qr factorization (and hence householder)
// This behaves the same as QR in matlab: [Q,R] = qr(A), except for signs
/* ************************************************************************* */
TEST( matrix, qr )
{
	double data[] = { -5, 0, 5, 0, 00, -5, 0, 5, 10, 0, 0, 0, 00, 10, 0, 0, 00,
			0, 0, -10, 10, 0, -10, 0 };
	Matrix A = Matrix_(6, 4, data);

	double dataQ[] = { -0.3333, 0, 0.2981, 0, 0, -0.8944, 0000000, -0.4472, 0,
			0.3651, -0.8165, 0, 00.6667, 0, 0.7454, 0, 0, 0, 0000000, 0.8944,
			0, 0.1826, -0.4082, 0, 0000000, 0, 0, -0.9129, -0.4082, 0, 00.6667,
			0, -0.5963, 0, 0, -0.4472, };
	Matrix expectedQ = Matrix_(6, 6, dataQ);

	double dataR[] = { 15, 0, -8.3333, 0, 00, 11.1803, 0, -2.2361, 00, 0,
			7.4536, 0, 00, 0, 0, 10.9545, 00, 0, 0, 0, 00, 0, 0, 0, };
	Matrix expectedR = Matrix_(6, 4, dataR);

	Matrix Q, R;
	boost::tie(Q, R) = qr(A);
	EXPECT(assert_equal(expectedQ, Q, 1e-4));
	EXPECT(assert_equal(expectedR, R, 1e-4));
	EXPECT(assert_equal(A, Q*R, 1e-14));
}

/* ************************************************************************* */
TEST( matrix, sub )
{
	double data1[] = { -5, 0, 5, 0, 0, 0, 00, -5, 0, 5, 0, 0, 10, 0, 0, 0, -10,
			0, 00, 10, 0, 0, 0, -10 };
	Matrix A = Matrix_(4, 6, data1);
	Matrix actual = sub(A, 1, 3, 1, 5);

	double data2[] = { -5, 0, 5, 0, 00, 0, 0, -10, };
	Matrix expected = Matrix_(2, 4, data2);

	EQUALITY(actual,expected);
}

/* ************************************************************************* */
TEST( matrix, trans )
{
	Matrix A = Matrix_(2, 2, 1.0, 3.0, 2.0, 4.0);
	Matrix B = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	EQUALITY(trans(A),B);
}

/* ************************************************************************* */
TEST( matrix, col_major_access )
{
	Matrix A = Matrix_(2, 2, 1.0, 2.0, 3.0, 4.0);
	const double* a = &A(0, 0);
	DOUBLES_EQUAL(2.0,a[2],1e-9);
}

/* ************************************************************************* */
TEST( matrix, weighted_elimination )
{
	// create a matrix to eliminate
	Matrix A = Matrix_(4, 6, -1., 0., 1., 0., 0., 0., 0., -1., 0., 1., 0., 0.,
			1., 0., 0., 0., -1., 0., 0., 1., 0., 0., 0., -1.);
	Vector b = Vector_(4, -0.2, 0.3, 0.2, -0.1);
	Vector sigmas = Vector_(4, 0.2, 0.2, 0.1, 0.1);

	// 	expected values
	Matrix expectedR = Matrix_(4, 6, 1., 0., -0.2, 0., -0.8, 0., 0., 1., 0.,
			-0.2, 0., -0.8, 0., 0., 1., 0., -1., 0., 0., 0., 0., 1., 0., -1.);
	Vector d = Vector_(4, 0.2, -0.14, 0.0, 0.2);
	Vector newSigmas = Vector_(4, 0.0894427, 0.0894427, 0.223607, 0.223607);

	Vector r;
	double di, sigma;
	size_t i;

	// perform elimination
	Matrix A1 = A;
	Vector b1 = b;
	std::list<boost::tuple<Vector, double, double> > solution =
			weighted_eliminate(A1, b1, sigmas);

	// unpack and verify
	i = 0;
	BOOST_FOREACH(boost::tie(r, di, sigma), solution){
		EXPECT(assert_equal(r, expectedR.row(i))); // verify r
		DOUBLES_EQUAL(d(i), di, 1e-8); // verify d
		DOUBLES_EQUAL(newSigmas(i), sigma, 1e-5); // verify sigma
		i += 1;
	}
}

/* ************************************************************************* */
TEST( matrix, inverse_square_root )
{
	Matrix measurement_covariance = Matrix_(3, 3, 0.25, 0.0, 0.0, 0.0, 0.25,
			0.0, 0.0, 0.0, 0.01);
	Matrix actual = inverse_square_root(measurement_covariance);

	Matrix expected = Matrix_(3, 3, 2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
			10.0);

	EQUALITY(expected,actual);
	EQUALITY(measurement_covariance,inverse(actual*actual));

	// Randomly generated test.  This test really requires inverse to
	// be working well; if it's not, there's the possibility of a
	// bug in inverse masking a bug in this routine since we
	// use the same inverse routing inside inverse_square_root()
	// as we use here to check it.

	Matrix M = Matrix_(5, 5,
			0.0785892, 0.0137923, -0.0142219, -0.0171880, 0.0028726,
			0.0137923, 0.0908911, 0.0020775, -0.0101952, 0.0175868,
			-0.0142219, 0.0020775, 0.0973051, 0.0054906, 0.0047064,
			-0.0171880,-0.0101952, 0.0054906, 0.0892453, -0.0059468,
			0.0028726, 0.0175868, 0.0047064, -0.0059468, 0.0816517);

	expected = Matrix_(5, 5,
			3.567126953241796, 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000,
			-0.590030436566913, 3.362022286742925, 0.000000000000000, 0.000000000000000, 0.000000000000000,
			0.618207860252376, -0.168166020746503, 3.253086082942785, 0.000000000000000, 0.000000000000000,
			0.683045380655496, 0.283773848115276, -0.099969232183396, 3.433537147891568, 0.000000000000000,
			-0.006740136923185, -0.669325697387650, -0.169716689114923, 0.171493059476284, 3.583921085468937);
	EQUALITY(expected, inverse_square_root(M));

}

/* *********************************************************************** */
// M was generated as the covariance of a set of random numbers.  L that
// we are checking against was generated via chol(M)' on octave
TEST( matrix, LLt )
{
	Matrix M = Matrix_(5, 5, 0.0874197, -0.0030860, 0.0116969, 0.0081463,
			0.0048741, -0.0030860, 0.0872727, 0.0183073, 0.0125325, -0.0037363,
			0.0116969, 0.0183073, 0.0966217, 0.0103894, -0.0021113, 0.0081463,
			0.0125325, 0.0103894, 0.0747324, 0.0036415, 0.0048741, -0.0037363,
			-0.0021113, 0.0036415, 0.0909464);

	Matrix expected = Matrix_(5, 5,
			0.295668226226627, 0.000000000000000, 0.000000000000000, 0.000000000000000, 0.000000000000000,
			-0.010437374483502, 0.295235094820875, 0.000000000000000, 0.000000000000000, 0.000000000000000,
			0.039560896175007, 0.063407813693827, 0.301721866387571, 0.000000000000000, 0.000000000000000,
			0.027552165831157, 0.043423266737274, 0.021695600982708, 0.267613525371710, 0.000000000000000,
			0.016485031422565, -0.012072546984405, -0.006621889326331, 0.014405837566082, 0.300462176944247);

	EQUALITY(expected, LLt(M));
}

/* ************************************************************************* */
TEST( matrix, multiplyAdd )
{
	Matrix A = Matrix_(3, 4, 4., 0., 0., 1., 0., 4., 0., 2., 0., 0., 1., 3.);
	Vector x = Vector_(4, 1., 2., 3., 4.), e = Vector_(3, 5., 6., 7.),
			expected = e + A * x;

	multiplyAdd(1, A, x, e);
	EXPECT(assert_equal(expected, e));
}

/* ************************************************************************* */
TEST( matrix, transposeMultiplyAdd )
{
	Matrix A = Matrix_(3, 4, 4., 0., 0., 1., 0., 4., 0., 2., 0., 0., 1., 3.);
	Vector x = Vector_(4, 1., 2., 3., 4.), e = Vector_(3, 5., 6., 7.),
			expected = x + trans(A) * e;

	transposeMultiplyAdd(1, A, e, x);
	EXPECT(assert_equal(expected, x));
}

/* ************************************************************************* */
TEST( matrix, linear_dependent )
{
	Matrix A = Matrix_(2, 3, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
	Matrix B = Matrix_(2, 3, -1.0, -2.0, -3.0, 8.0, 10.0, 12.0);
	EXPECT(linear_dependent(A, B));
}

/* ************************************************************************* */
TEST( matrix, linear_dependent2 )
{
	Matrix A = Matrix_(2, 3, 0.0, 2.0, 3.0, 4.0, 5.0, 6.0);
	Matrix B = Matrix_(2, 3, 0.0, -2.0, -3.0, 8.0, 10.0, 12.0);
	EXPECT(linear_dependent(A, B));
}

/* ************************************************************************* */
TEST( matrix, linear_dependent3 )
{
	Matrix A = Matrix_(2, 3, 0.0, 2.0, 3.0, 4.0, 5.0, 6.0);
	Matrix B = Matrix_(2, 3, 0.0, -2.0, -3.0, 8.1, 10.0, 12.0);
	EXPECT(linear_independent(A, B));
}

/* ************************************************************************* */
TEST( matrix, svd1 )
{
	Vector v = Vector_(3, 2., 1., 0.);
	Matrix U1 = eye(4, 3), S1 = diag(v), V1 = eye(3, 3), A = (U1 * S1)
			* Matrix(trans(V1));
	Matrix U, V;
	Vector s;
	svd(A, U, s, V);
	Matrix S = diag(s);
	EXPECT(assert_equal(U*S*Matrix(trans(V)),A));
	EXPECT(assert_equal(S,S1));
}

/* ************************************************************************* */
/// Sample A matrix for SVD
static Matrix sampleA = Matrix_(3, 2, 0.,-2., 0., 0., 3., 0.);
static Matrix sampleAt = trans(sampleA);

/* ************************************************************************* */
TEST( matrix, svd2 )
{
	Matrix U, V;
	Vector s;

	Matrix expectedU = Matrix_(3, 2, 0.,-1.,0.,0.,1.,0.);
	Vector expected_s = Vector_(2, 3.,2.);
	Matrix expectedV = Matrix_(2, 2, 1.,0.,0.,1.);

	svd(sampleA, U, s, V);

	EXPECT(assert_equal(expectedU,U));
	EXPECT(assert_equal(expected_s,s,1e-9));
	EXPECT(assert_equal(expectedV,V));
}

/* ************************************************************************* */
TEST( matrix, svd3 )
{
	Matrix U, V;
	Vector s;

	Matrix expectedU = Matrix_(2, 2, -1.,0.,0.,-1.);
	Vector expected_s = Vector_(2, 3.0,2.0);
	Matrix expectedV = Matrix_(3, 2, 0.,1.,0.,0.,-1.,0.);

	svd(sampleAt, U, s, V);
	Matrix S = diag(s);
	Matrix t = U * S;
	Matrix Vt = trans(V);

	EXPECT(assert_equal(sampleAt, prod(t, Vt)));
	EXPECT(assert_equal(expectedU,U));
	EXPECT(assert_equal(expected_s,s,1e-9));
	EXPECT(assert_equal(expectedV,V));
}

/* ************************************************************************* */
TEST( matrix, svd4 )
{
  Matrix U, V;
  Vector s;

  Matrix A = Matrix_(3,2,
      0.8147,    0.9134,
      0.9058,    0.6324,
      0.1270,    0.0975);

  Matrix expectedU = Matrix_(3,2,
     0.7397,   0.6724,
     0.6659,   -0.7370,
     0.0970,   -0.0689);

  Vector expected_s = Vector_(2, 1.6455, 0.1910);

  Matrix expectedV = Matrix_(2,2,
     0.7403,   -0.6723,
     0.6723,   0.7403);

  svd(A, U, s, V);
  Matrix reconstructed = U * diag(s) * trans(V);

  EXPECT(assert_equal(A, reconstructed, 1e-4));
  EXPECT(assert_equal(expectedU,U, 1e-3));
  EXPECT(assert_equal(expected_s,s, 1e-4));
  EXPECT(assert_equal(expectedV,V, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, DLT )
{
  Matrix A = Matrix_(8,9,
      0.21,        -0.42,       -10.71,         0.18,        -0.36,        -9.18,        -0.61,         1.22,        31.11,
      0.44,        -0.66,       -15.84,         0.34,        -0.51,       -12.24,        -1.64,         2.46,        59.04,
      0.69,        -8.28,       -12.19,        -0.48,         5.76,         8.48,        -1.89,        22.68,        33.39,
      0.96,         -8.4,       -17.76,         -0.6,         5.25,         11.1,        -3.36,         29.4,        62.16,
      1.25,          0.3,         2.75,         -3.5,        -0.84,         -7.7,        16.25,          3.9,        35.75,
      1.56,         0.42,         4.56,        -3.38,        -0.91,        -9.88,        22.36,         6.02,        65.36,
      1.89,         2.24,         3.99,         3.24,         3.84,         6.84,        18.09,        21.44,        38.19,
      2.24,         2.48,         6.24,         3.08,         3.41,         8.58,        24.64,        27.28,        68.64
  );
  int rank;
  double error;
  Vector actual;
  boost::tie(rank,error,actual) = DLT(A);
  Vector expected = Vector_(9, -0.0, 0.2357, 0.4714, -0.2357, 0.0, - 0.4714,-0.4714, 0.4714, 0.0);
  EXPECT_LONGS_EQUAL(8,rank);
  EXPECT_DOUBLES_EQUAL(0,error,1e-8);
  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
