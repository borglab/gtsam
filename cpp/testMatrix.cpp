/**
 * @file   testMatrix.cpp
 * @brief  Unit test for Matrix Library
 * @author Christian Potthast
 * @author Carlos Nieto
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include "Matrix.h"
#include "NoiseModel.h"

using namespace std;
using namespace gtsam;

static double inf = std::numeric_limits<double>::infinity();

/* ************************************************************************* */
TEST( matrix, constructor_data )
{
  double data[] = {-5, 3,
                    0, -5 };
  Matrix A = Matrix_(2,2,data);

  Matrix B(2,2);
  B(0,0) = -5 ; B(0,1) =  3;
  B(1,0) =  0 ; B(1,1) = -5;

  EQUALITY(A,B);
}

/* ************************************************************************* */

TEST( matrix, constructor_vector )
{
  double data[] = {-5, 3,
                    0, -5 };
  Matrix A = Matrix_(2,2,data);
  Vector v(4); copy(data,data+4,v.begin());
  Matrix B = Matrix_(2,2,v); // this one is column order !
  EQUALITY(A,trans(B));
}

/* ************************************************************************* */
TEST( matrix, Matrix_ )
{
  Matrix A = Matrix_(2,2, 
		       -5.0 , 3.0,
		       00.0, -5.0 );
  Matrix B(2,2);
  B(0,0) = -5 ; B(0,1) =  3;
  B(1,0) =  0 ; B(1,1) = -5;

  EQUALITY(A,B);

}

/* ************************************************************************* */
TEST( matrix, row_major )
{
  Matrix A = Matrix_(2,2, 
		       1.0, 2.0,
		       3.0, 4.0 );
  const double * const a = &A(0,0);
  CHECK(a[0] == 1);
  CHECK(a[1] == 2);
  CHECK(a[2] == 3);
  CHECK(a[3] == 4);
}

/* ************************************************************************* */
TEST( matrix, collect1 )
{
	Matrix A = Matrix_(2,2,
			-5.0 , 3.0,
			00.0, -5.0 );
	Matrix B = Matrix_(2,3,
			-0.5 , 2.1, 1.1,
			3.4 , 2.6 , 7.1);
	Matrix AB = collect(2, &A, &B);
	Matrix C(2,5);
	for(int i = 0; i < 2; i++) for(int j = 0; j < 2; j++) C(i,j) = A(i,j);
	for(int i = 0; i < 2; i++) for(int j = 0; j < 3; j++) C(i,j+2) = B(i,j);

	EQUALITY(C,AB);

}

/* ************************************************************************* */
TEST( matrix, collect2 )
{
	Matrix A = Matrix_(2,2,
			-5.0 , 3.0,
			00.0, -5.0 );
	Matrix B = Matrix_(2,3,
			-0.5 , 2.1, 1.1,
			3.4 , 2.6 , 7.1);
	vector<const Matrix*> matrices;
	matrices.push_back(&A);
	matrices.push_back(&B);
	Matrix AB = collect(matrices);
	Matrix C(2,5);
	for(int i = 0; i < 2; i++) for(int j = 0; j < 2; j++) C(i,j) = A(i,j);
	for(int i = 0; i < 2; i++) for(int j = 0; j < 3; j++) C(i,j+2) = B(i,j);

	EQUALITY(C,AB);

}

/* ************************************************************************* */
TEST( matrix, collect3 )
{
	Matrix A, B;
	A = eye(2,3);
	B = eye(2,3);
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
  Matrix A = Matrix_(2,2, 
		       -5.0 , 3.0,
		       00.0, -5.0 );
  Matrix B = Matrix_(3,2,
		       -0.5 , 2.1,
		       1.1, 3.4 ,
		       2.6 , 7.1);
  Matrix AB = stack(2, &A, &B);
  Matrix C(5,2);
  for(int i = 0; i < 2; i++) for(int j = 0; j < 2; j++) C(i,j) = A(i,j);
  for(int i = 0; i < 3; i++) for(int j = 0; j < 2; j++) C(i+2,j) = B(i,j);

  EQUALITY(C,AB);
}

/* ************************************************************************* */
TEST( matrix, column )
{
	Matrix A = Matrix_(4, 7,
	   -1.,  0.,  1.,  0.,  0.,  0., -0.2,
		0., -1.,  0.,  1.,  0.,  0.,  0.3,
		1.,  0.,  0.,  0., -1.,  0.,  0.2,
		0.,  1.,  0.,  0.,  0., -1., -0.1);
	Vector a1 = column_(A, 0);
	Vector exp1 = Vector_(4, -1., 0., 1., 0.);
	CHECK(assert_equal(a1, exp1));

	Vector a2 = column_(A, 3);
	Vector exp2 = Vector_(4,  0., 1., 0., 0.);
	CHECK(assert_equal(a2, exp2));

	Vector a3 = column_(A, 6);
	Vector exp3 = Vector_(4, -0.2, 0.3, 0.2, -0.1);
	CHECK(assert_equal(a3, exp3));
}

/* ************************************************************************* */
TEST( matrix, row )
{
	Matrix A = Matrix_(4, 7,
	   -1.,  0.,  1.,  0.,  0.,  0., -0.2,
		0., -1.,  0.,  1.,  0.,  0.,  0.3,
		1.,  0.,  0.,  0., -1.,  0.,  0.2,
		0.,  1.,  0.,  0.,  0., -1., -0.1);
	Vector a1 = row(A, 0);
	Vector exp1 = Vector_(7, -1.,  0.,  1.,  0.,  0.,  0., -0.2);
	CHECK(assert_equal(a1, exp1));

	Vector a2 = row(A, 2);
	Vector exp2 = Vector_(7, 1.,  0.,  0.,  0., -1.,  0.,  0.2);
	CHECK(assert_equal(a2, exp2));

	Vector a3 = row(A, 3);
	Vector exp3 = Vector_(7, 0.,  1.,  0.,  0.,  0., -1., -0.1);
	CHECK(assert_equal(a3, exp3));
}

/* ************************************************************************* */
TEST( matrix, zeros )
{
  Matrix A(2,3);
  A(0,0) = 0 ; A(0,1) = 0; A(0,2) = 0;
  A(1,0) = 0 ; A(1,1) = 0; A(1,2) = 0;

  Matrix zero = zeros(2,3);

  EQUALITY(A , zero);
}

/* ************************************************************************* */
TEST( matrix, scale_columns )
{
	Matrix A(3,4);
	A(0,0) = 1.; A(0,1) = 1.; A(0,2)= 1.; A(0,3)= 1.;
	A(1,0) = 1.; A(1,1) = 1.; A(1,2)= 1.; A(1,3)= 1.;
	A(2,0) = 1.; A(2,1) = 1.; A(2,2)= 1.; A(2,3)= 1.;

	Vector v = Vector_(4, 2., 3., 4., 5.);

	Matrix actual = vector_scale(A,v);

	Matrix expected(3,4);
	expected(0,0) = 2.; expected(0,1) = 3.; expected(0,2)= 4.; expected(0,3)= 5.;
	expected(1,0) = 2.; expected(1,1) = 3.; expected(1,2)= 4.; expected(1,3)= 5.;
	expected(2,0) = 2.; expected(2,1) = 3.; expected(2,2)= 4.; expected(2,3)= 5.;

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( matrix, scale_rows )
{
	Matrix A(3,4);
	A(0,0) = 1.; A(0,1) = 1.; A(0,2)= 1.; A(0,3)= 1.;
	A(1,0) = 1.; A(1,1) = 1.; A(1,2)= 1.; A(1,3)= 1.;
	A(2,0) = 1.; A(2,1) = 1.; A(2,2)= 1.; A(2,3)= 1.;

	Vector v = Vector_(3, 2., 3., 4.);

	Matrix actual = vector_scale(v,A);

	Matrix expected(3,4);
	expected(0,0) = 2.; expected(0,1) = 2.; expected(0,2)= 2.; expected(0,3)= 2.;
	expected(1,0) = 3.; expected(1,1) = 3.; expected(1,2)= 3.; expected(1,3)= 3.;
	expected(2,0) = 4.; expected(2,1) = 4.; expected(2,2)= 4.; expected(2,3)= 4.;

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
TEST( matrix, equal )
{
  Matrix A(4,4);
  A(0,0) = -1; A(0,1) = 1; A(0,2)= 2; A(0,3)= 3;
  A(1,0) =  1; A(1,1) =-3; A(1,2)= 1; A(1,3)= 3;
  A(2,0) =  1; A(2,1) = 2; A(2,2)=-1; A(2,3)= 4;
  A(3,0) =  2; A(3,1) = 1; A(3,2)= 2; A(3,3)=-2;

  Matrix A2(A);

  Matrix A3(A);
  A3(3,3)=-2.1;

  CHECK(A==A2);
  CHECK(A!=A3);
}

/* ************************************************************************* */
TEST( matrix, equal_nan )
{
  Matrix A(4,4);
  A(0,0) = -1; A(0,1) = 1; A(0,2)= 2; A(0,3)= 3;
  A(1,0) =  1; A(1,1) =-3; A(1,2)= 1; A(1,3)= 3;
  A(2,0) =  1; A(2,1) = 2; A(2,2)=-1; A(2,3)= 4;
  A(3,0) =  2; A(3,1) = 1; A(3,2)= 2; A(3,3)=-2;

  Matrix A2(A);

  Matrix A3(A);
  A3(3,3)=inf;

  CHECK(A!=A3);
}

/* ************************************************************************* */
TEST( matrix, addition )
{
  Matrix A = Matrix_(2,2, 
		       1.0, 2.0,
		       3.0, 4.0);
  Matrix B = Matrix_(2,2, 
		       4.0, 3.0,
		       2.0, 1.0);
  Matrix C = Matrix_(2,2, 
		       5.0, 5.0,
		       5.0, 5.0);
  EQUALITY(A+B,C);
}

/* ************************************************************************* */
TEST( matrix, addition_in_place )
{
  Matrix A = Matrix_(2,2, 
		       1.0, 2.0,
		       3.0, 4.0);
  Matrix B = Matrix_(2,2, 
		       4.0, 3.0,
		       2.0, 1.0);
  Matrix C = Matrix_(2,2, 
		       5.0, 5.0,
		       5.0, 5.0);
  A += B;
  EQUALITY(A,C);
}

/* ************************************************************************* */
TEST( matrix, subtraction )
{
  Matrix A = Matrix_(2,2, 
		       1.0, 2.0,
		       3.0, 4.0);
  Matrix B = Matrix_(2,2, 
		       4.0, 3.0,
		       2.0, 1.0);
  Matrix C = Matrix_(2,2, 
		       -3.0, -1.0,
		        1.0,  3.0);
  EQUALITY(A-B,C);
}

/* ************************************************************************* */
TEST( matrix, subtraction_in_place )
{
  Matrix A = Matrix_(2,2, 
		       1.0, 2.0,
		       3.0, 4.0);
  Matrix B = Matrix_(2,2, 
		       4.0, 3.0,
		       2.0, 1.0);
  Matrix C = Matrix_(2,2, 
		       -3.0, -1.0,
		        1.0,  3.0);
  A -= B;
  EQUALITY(A,C);
}

/* ************************************************************************* */
TEST( matrix, multiplication )
{
  Matrix A(2,2);
  A(0,0) = -1; A(1,0) = 1;
  A(0,1) =  1; A(1,1) =-3;

  Matrix B(2,1);
  B(0,0) = 1.2;
  B(1,0) = 3.4;

  Matrix AB(2,1);
  AB(0,0) = 2.2;
  AB(1,0) = -9.;

  EQUALITY(A*B,AB);
}

/* ************************************************************************* */
TEST( matrix, scalar_matrix_multiplication )
{
  Vector result(2);

  Matrix A(2,2);
  A(0,0) = -1; A(1,0) = 1;
  A(0,1) =  1; A(1,1) =-3;

  Matrix B(2,2);
  B(0,0) = -10; B(1,0) = 10;
  B(0,1) =  10; B(1,1) =-30;

  EQUALITY((10*A),B);
}

/* ************************************************************************* */
TEST( matrix, matrix_vector_multiplication )
{
  Vector result(2);

  Matrix A = Matrix_(2,3,
		       1.0,2.0,3.0,
		       4.0,5.0,6.0
		       );
  Vector v = Vector_(3,1.,2.,3.);
  Vector Av = Vector_(2,14.,32.);
  Vector AtAv = Vector_(3,142.,188.,234.);

  EQUALITY(A*v,Av);
  EQUALITY(A^Av,AtAv);
}

/* ************************************************************************* */
TEST( matrix, nrRowsAndnrCols )
{
  Matrix A(3,6);
  LONGS_EQUAL( A.size1() , 3 );
  LONGS_EQUAL( A.size2() , 6 );
}


/* ************************************************************************* */
TEST( matrix, scalar_divide )
{
  Matrix A(2,2);
  A(0,0) = 10; A(1,0) = 30;
  A(0,1) = 20; A(1,1) = 40;

  Matrix B(2,2);
  B(0,0) = 1; B(1,0) = 3;
  B(0,1) = 2; B(1,1) = 4;

  EQUALITY(B,A/10);
}

/* ************************************************************************* */
TEST( matrix, inverse )
{
  Matrix A(3,3);
  A(0,0)= 1;  A(0,1)=2; A(0,2)=3;
  A(1,0)= 0;  A(1,1)=4; A(1,2)=5;
  A(2,0)= 1;  A(2,1)=0; A(2,2)=6;

  Matrix Ainv = inverse(A);

  Matrix expected(3,3);
  expected(0,0)= 1.0909;   expected(0,1)=-0.5454; expected(0,2)=-0.0909;
  expected(1,0)= 0.2272;   expected(1,1)= 0.1363; expected(1,2)=-0.2272;
  expected(2,0)= -0.1818;  expected(2,1)= 0.0909; expected(2,2)=0.1818;

  CHECK(assert_equal(expected, Ainv, 1e-4));
}

/* ************************************************************************* */
TEST( matrix, backsubtitution )
{
	// TEST ONE  2x2 matrix U1*x=b1
	Vector expected1 = Vector_(2, 3.6250, -0.75);
	Matrix U22 = Matrix_(2, 2,
			2., 3.,
			0., 4.);
	Vector b1 = U22*expected1;
	CHECK( assert_equal(expected1 , backSubstituteUpper(U22, b1), 0.000001));

	// TEST TWO  3x3 matrix U2*x=b2
	Vector expected2 = Vector_(3, 5.5, -8.5, 5.);
	Matrix U33 = Matrix_(3, 3,
			3., 5., 6.,
			0., 2., 3.,
			0., 0., 1.);
	Vector b2 = U33*expected2;
	CHECK( assert_equal(expected2 , backSubstituteUpper(U33, b2), 0.000001));

	// TEST THREE  Lower triangular 3x3 matrix L3*x=b3
	Vector expected3 = Vector_(3, 1., 1., 1.);
	Matrix L3 = trans(U33);
	Vector b3 = L3*expected3;
	CHECK( assert_equal(expected3 , backSubstituteLower(L3, b3), 0.000001));

	// TEST FOUR Try the above with transpose backSubstituteUpper
	CHECK( assert_equal(expected3 , backSubstituteUpper(b3,U33), 0.000001));
}

/* ************************************************************************* */
// unit tests for housholder transformation 
/* ************************************************************************* */
TEST( matrix, houseHolder )
{
  double data[] = {-5,  0, 5, 0,  0,  0,  -1,
		   00, -5, 0, 5,  0,  0, 1.5,
		   10,  0, 0, 0,-10,  0,   2,
		   00, 10, 0, 0,  0,-10,  -1};

  // check in-place householder, with v vectors below diagonal
  double data1[] = {
  		11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236,
  		0, 11.1803, 0, -2.2361, 0, -8.9443, -1.565,
  		-0.618034, 0, 4.4721, 0, -4.4721, 0, 0,
			0, -0.618034, 0, 4.4721, 0, -4.4721, 0.894 };
  Matrix expected1 = Matrix_(4,7, data1);
  Matrix A1 = Matrix_(4, 7, data);
  householder_(A1,3);
  CHECK(assert_equal(expected1, A1, 1e-3));

  // in-place, with zeros below diagonal
  double data2[] = {
  		11.1803, 0, -2.2361, 0, -8.9443, 0, 2.236,
  		0, 11.1803, 0, -2.2361, 0, -8.9443, -1.565,
  		0, 0, 4.4721, 0, -4.4721, 0, 0,
  		0, 0, 0, 4.4721, 0, -4.4721, 0.894 };
  Matrix expected = Matrix_(4,7, data2);
  Matrix A2 = Matrix_(4, 7, data);
  householder(A2,3);  
  CHECK(assert_equal(expected, A2, 1e-3));
}
/* ************************************************************************* */
// unit test for qr factorization (and hence householder)
// This behaves the same as QR in matlab: [Q,R] = qr(A), except for signs
/* ************************************************************************* */
TEST( matrix, qr )
{
  double data[] = {-5,  0,  5,  0, 
		   00, -5,  0,  5, 
		   10,  0,  0,  0, 
		   00, 10,  0,  0,
		   00,  0,  0,-10,
		   10,  0,-10,  0};
  Matrix A = Matrix_(6, 4, data);

  double dataQ[] = {
    -0.3333,         0,    0.2981,         0,         0,   -0.8944,
    0000000,   -0.4472,         0,    0.3651,   -0.8165,         0,
    00.6667,         0,    0.7454,         0,         0,         0,
    0000000,    0.8944,         0,    0.1826,   -0.4082,         0,
    0000000,         0,         0,   -0.9129,   -0.4082,         0,
    00.6667,         0,   -0.5963,         0,         0,   -0.4472,
  };
  Matrix expectedQ = Matrix_(6,6, dataQ);
  
  double dataR[] = {
    15,        0,   -8.3333,         0,
    00,  11.1803,         0,   -2.2361,
    00,        0,    7.4536,         0,
    00,        0,         0,   10.9545,
    00,        0,         0,         0,
    00,        0,         0,         0,
  };
  Matrix expectedR = Matrix_(6,4, dataR);

  Matrix Q,R;
  boost::tie(Q,R) = qr(A);
  CHECK(assert_equal(expectedQ, Q,  1e-4));
  CHECK(assert_equal(expectedR, R, 1e-4));
  CHECK(assert_equal(A, Q*R, 1e-14));
}

/* ************************************************************************* */
TEST( matrix, sub )
{
  double data1[] = {
    -5,  0, 5, 0,  0,  0,
    00, -5, 0, 5,  0,  0,
    10,  0, 0, 0,-10,  0,
    00, 10, 0, 0,  0,-10
  };
  Matrix A = Matrix_(4,6, data1);
  Matrix actual = sub(A,1,3,1,5);

  double data2[] = {
    -5, 0, 5,  0,
    00, 0, 0,-10,
  };
  Matrix expected = Matrix_(2,4, data2);

  EQUALITY(actual,expected);
}


/* ************************************************************************* */
TEST( matrix, trans )
{
  Matrix A = Matrix_(2,2, 
		       1.0 ,3.0,
		       2.0, 4.0 );
  Matrix B = Matrix_(2,2, 
		       1.0 ,2.0,
		       3.0, 4.0 );
  EQUALITY(trans(A),B);
}

/* ************************************************************************* */
TEST( matrix, row_major_access )
{
  Matrix A = Matrix_(2,2,1.0,2.0,3.0,4.0);
  const double* a = &A(0,0);
  DOUBLES_EQUAL(3,a[2],1e-9);
}

/* ************************************************************************* */
TEST( matrix, svd )
{ 
  double data[] = {2,1,0};
  Vector v(3); copy(data,data+3,v.begin());
  Matrix U1=eye(4,3), S1=diag(v), V1=eye(3,3), A=(U1*S1)*Matrix(trans(V1));
  Matrix U,V;
  Vector s;
  svd(A,U,s,V);
  Matrix S=diag(s);
  EQUALITY(U*S*Matrix(trans(V)),A);
  EQUALITY(S,S1);
}

/* ************************************************************************* */
// update A, b
// A' \define A_{S}-ar and b'\define b-ad
// __attribute__ ((noinline))	// uncomment to prevent inlining when profiling
static void updateAb(Matrix& A, Vector& b, int j, const Vector& a,
		const Vector& r, double d) {
	const size_t m = A.size1(), n = A.size2();
	for (int i = 0; i < m; i++) { // update all rows
		double ai = a(i);
		b(i) -= ai * d;
		double *Aij = A.data().begin() + i * n + j + 1;
		const double *rptr = r.data().begin() + j + 1;
		// A(i,j+1:end) -= ai*r(j+1:end)
		for (int j2 = j + 1; j2 < n; j2++, Aij++, rptr++)
			*Aij -= ai * (*rptr);
	}
}

/* ************************************************************************* */
list<boost::tuple<Vector, double, double> >
weighted_eliminate2(Matrix& A, Vector& b, const sharedGaussian& model) {
	size_t m = A.size1(), n = A.size2(); // get size(A)
	size_t maxRank = min(m,n);

	// pre-whiten everything
	model->WhitenInPlace(A);
	b = model->whiten(b);

	// create list
	list<boost::tuple<Vector, double, double> > results;

	// We loop over all columns, because the columns that can be eliminated
	// are not necessarily contiguous. For each one, estimate the corresponding
	// scalar variable x as d-rS, with S the separator (remaining columns).
	// Then update A and b by substituting x with d-rS, zero-ing out x's column.
	for (int j=0; j<n; ++j) {
		// extract the first column of A
		Vector a(column(A, j)); // ublas::matrix_column is slower !

		// Calculate weighted pseudo-inverse and corresponding precision
		double precision = dot(a,a);
		Vector pseudo = a/precision;

		// if precision is zero, no information on this column
		if (precision < 1e-8) continue;

		// create solution and copy into r
		Vector r(basis(n, j));
		for (int j2=j+1; j2<n; ++j2) // expensive !!
			r(j2) = inner_prod(pseudo, boost::numeric::ublas::matrix_column<Matrix>(A, j2));

		// create the rhs
		double d = inner_prod(pseudo, b);

		// construct solution (r, d, sigma)
		// TODO: avoid sqrt, store precision or at least variance
		results.push_back(boost::make_tuple(r, d, 1./sqrt(precision)));

		// exit after rank exhausted
		if (results.size()>=maxRank) break;

		// update A, b, expensive, suing outer product
		// A' \define A_{S}-a*r and b'\define b-d*a
		updateAb(A, b, j, a, r, d);
	}

	return results;
}

/* ************************************************************************* */
void weighted_eliminate3(Matrix& A, Vector& b, const sharedGaussian& model) {
	size_t m = A.size1(), n = A.size2(); // get size(A)
	size_t maxRank = min(m,n);

	// pre-whiten everything
	model->WhitenInPlace(A);
	b = model->whiten(b);

	householder_(A, maxRank);
}

/* ************************************************************************* */
TEST( matrix, weighted_elimination )
{
	// create a matrix to eliminate
	Matrix A = Matrix_(4, 6,
		   -1.,  0.,  1.,  0.,  0.,  0.,
		    0., -1.,  0.,  1.,  0.,  0.,
	      1.,  0.,  0.,  0., -1.,  0.,
	      0.,  1.,  0.,  0.,  0., -1.);
	Vector b = Vector_(4, -0.2, 0.3, 0.2, -0.1);
	Vector sigmas = Vector_(4, 0.2, 0.2, 0.1, 0.1);

	// 	expected values
	Matrix expectedR = Matrix_(4, 6,
			1.,  0., -0.2,  0., -0.8, 0.,
			0.,  1.,  0.,-0.2,   0., -0.8,
			0.,  0.,  1.,   0., -1.,  0.,
			0.,  0.,  0.,   1.,  0., -1.);
	Vector d = Vector_(4, 0.2, -0.14, 0.0, 0.2);
	Vector newSigmas  = Vector_(4,
			0.0894427,
			0.0894427,
			0.223607,
			0.223607);

	Vector r; double di, sigma;
	size_t i;

	// perform elimination
	Matrix A1 = A; Vector b1 = b;
	std::list<boost::tuple<Vector, double, double> > solution =
								weighted_eliminate(A1, b1, sigmas);

	// unpack and verify
	i=0;
	BOOST_FOREACH(boost::tie(r, di, sigma), solution) {
		CHECK(assert_equal(r, row(expectedR, i))); // verify r
		DOUBLES_EQUAL(d(i), di, 1e-8);             // verify d
		DOUBLES_EQUAL(newSigmas(i), sigma, 1e-5);  // verify sigma
		i += 1;
	}

	// perform elimination with NoiseModel
	Matrix A2 = A; Vector b2 = b;
	sharedGaussian model = noiseModel::Diagonal::Sigmas(sigmas);
	std::list<boost::tuple<Vector, double, double> > solution2 =
								weighted_eliminate2(A2, b2, model);

	// unpack and verify
	i=0;
	BOOST_FOREACH(boost::tie(r, di, sigma), solution2) {
		CHECK(assert_equal(r, row(expectedR, i))); // verify r
		DOUBLES_EQUAL(d(i), di, 1e-8);             // verify d
		DOUBLES_EQUAL(newSigmas(i), sigma, 1e-5);  // verify sigma
		i += 1;
	}

	// perform elimination with NoiseModel
	weighted_eliminate3(A, b, model);
	sharedGaussian newModel = noiseModel::Diagonal::Sigmas(newSigmas);
//	print(A);
//	print(newModel->Whiten(expectedR));
}

/* ************************************************************************* */
TEST( matrix, inverse_square_root )
{
	Matrix measurement_covariance = Matrix_(3,3,
			0.25, 0.0, 0.0,
			0.0, 0.25, 0.0,
			0.0, 0.0, 0.01
			);
	Matrix actual = inverse_square_root(measurement_covariance);

	Matrix expected = Matrix_(3,3,
			2.0, 0.0, 0.0,
			0.0, 2.0, 0.0,
			0.0, 0.0, 10.0
			);

	EQUALITY(expected,actual);
	EQUALITY(measurement_covariance,inverse(actual*actual));
}

/* ************************************************************************* */
TEST( matrix, square_root_positive )
{
  Matrix cov = Matrix_(3,3,
			4.0, 0.0, 0.0,
			0.0, 4.0, 0.0,
			0.0, 0.0, 100.0
      );

  Matrix expected = Matrix_(3,3,
			2.0, 0.0, 0.0,
			0.0, 2.0, 0.0,
			0.0, 0.0, 10.0
			);

  Matrix actual = square_root_positive(cov);
  CHECK(assert_equal(expected, actual));
  CHECK(assert_equal(cov, prod(trans(actual),actual)));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
