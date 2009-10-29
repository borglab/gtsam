/**
 * @file   testMatrix.cpp
 * @brief  Unit test for Matrix Library
 * @author Christian Potthast
 * @author Carlos Nieto
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>
#include "Matrix.h"

using namespace std;
using namespace gtsam;

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
TEST( matrix, collect )
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
TEST( matrix, zeros )
{
  Matrix A(2,3);
  A(0,0) = 0 ; A(0,1) = 0; A(0,2) = 0;
  A(1,0) = 0 ; A(1,1) = 0; A(1,2) = 0;

  Matrix zero = zeros(2,3);

  EQUALITY(A , zero);
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
  Vector v(3);
  v(0) = 1.0;
  v(1) = 2.0;
  v(2) = 3.0;

  Vector Av(2);
  Av(0) = 14.0;
  Av(1) = 32.0;

  EQUALITY(A*v,Av);
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
/* unit test for backsubstitution                                           */
/* ************************************************************************* */
TEST( matrix, backsubtitution )
{
  // TEST ONE  2x2 matrix
  Vector expectedA(2);
  expectedA(0) = 3.6250 ; expectedA(1) = -0.75;

  // create a 2x2 matrix
  double dataA[] = {2, 3,
                    0, 4 };
  Matrix A = Matrix_(2,2,dataA);
  Vector Ab(2); Ab(0) = 5; Ab(1) = -3;

  CHECK( assert_equal(expectedA , backsubstitution(A, Ab), 0.000001));

  // TEST TWO  3x3 matrix
  Vector expectedB(3);
  expectedB(0) = 5.5 ; expectedB(1) = -8.5; expectedB(2) = 5;


  // create a 3x3 matrix
  double dataB[] = { 3, 5, 6,
                     0, 2, 3,
                     0, 0, 1 };
  Matrix B = Matrix_(3,3,dataB);

  Vector Bb(3);
  Bb(0) = 4;	Bb(1) = -2; Bb(2) = 5;

  CHECK( assert_equal(expectedB , backsubstitution(B, Bb), 0.000001));
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
  double data1[] = {0011.1803,        0, -2.2361,       0, -8.9443,       0,  2.236,
		    000000000,  11.1803,       0, -2.2361,       0, -8.9443, -1.565,
		    -0.618034,        0,  4.4721,       0, -4.4721,       0,  0,
		    000000000,-0.618034,       0,  4.4721,       0, -4.4721,  0.894};
  Matrix expected1 = Matrix_(4,7, data1);
  Matrix A1 = Matrix_(4, 7, data);
  householder_(A1,3);
  CHECK(assert_equal(expected1, A1, 1e-3));

  // in-place, with zeros below diagonal
  double data2[] = {0011.1803,        0, -2.2361,       0, -8.9443,       0,  2.236,
		    000000000,  11.1803,       0, -2.2361,       0, -8.9443, -1.565, 
		    000000000,        0,  4.4721,       0, -4.4721,       0,  0,
		    000000000,        0,       0,  4.4721,       0, -4.4721,  0.894};
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
TEST( matrix, whouse_subs )
{
	// create the system
	Matrix A(2,2);
	A(0,0) = 1.0; A(0,1) = 3.0;
	A(1,0) = 2.0; A(1,1) = 4.0;

	// Vector to eliminate
	Vector a(2);
	a(0) = 1.0; a(1) = 2.0;

	Vector tau(2); //correspond to sigmas = [0.1 0.2]
	tau(0) = 100.; tau(1) = 25.;

	// find the pseudoinverse
	Vector pseudo = whouse_solve(a, tau);

	// substitute
	int row = 0; // eliminating the first column
	whouse_subs(A, row, a, pseudo);

	// create expected value
	Matrix exp(2,2);
	exp(0,0) = 1.0; exp(0,1) = 3.0;
	exp(1,0) = 0.0; exp(1,1) = -1.0;

	// verify
	CHECK(assert_equal(A, exp));
}
/* ************************************************************************* */
TEST( matrix, whouse_subs_multistep )
{
	// update two matrices
	double sigma1 = 0.2; double tau1 = 1/(sigma1*sigma1);
	double sigma2 = 0.1; double tau2 = 1/(sigma2*sigma2);

	Matrix Ax2 = Matrix_(4,2,
			// x2
			-1., 0.,
			+0.,-1.,
			1., 0.,
			+0.,1.
	);

	Matrix Al1 = Matrix_(4,2,
			// l1
			1., 0.,
			0., 1.,
			0., 0.,
			0., 0.
	);

	// Eliminating x2 - step 1
	Vector a1 = Vector_(4, -1., 0., 1., 0.);
	Vector tau = Vector_(4, tau1, tau1, tau2, tau2);
	Vector pseudo1 = whouse_solve(a1, tau);

	size_t row = 0;
	whouse_subs(Ax2, row, a1, pseudo1);
	whouse_subs(Al1, row, a1, pseudo1);

	// verify first update
	Matrix Ax2_exp = Matrix_(4,2,
			-1., 0.,
			+0.,-1.,
			+0., 0.,
			+0.,1.
	);
	CHECK(assert_equal(Ax2, Ax2_exp, 1e-4));
	Matrix Al1_exp = Matrix_(4,2,
				// l1
				1., 0.,
				0., 1.,
				0.2, 0.,
				0., 0.
	);
	CHECK(assert_equal(Al1, Al1_exp, 1e-4));

	// Eliminating x2 - step 2
	Vector a2 = Vector_(3, -1., 0., 1.);
	Vector tauR2 = sub(tau,1,4);
	Vector pseudo2 = whouse_solve(a2, tauR2);

	row = 1;
	whouse_subs(Ax2, row, a2, pseudo2);
	whouse_subs(Al1, row, a2, pseudo2);

	// verify second update
	Ax2_exp = Matrix_(4,2,
			-1., 0.,
			+0.,-1.,
			+0., 0.,
			+0., 0.
	);
	CHECK(assert_equal(Ax2, Ax2_exp, 1e-4));
	Al1_exp = Matrix_(4,2,
			1., 0.,
			0., 1.,
			0.2, 0.,
			0., 0.2
	);
	CHECK(assert_equal(Al1, Al1_exp, 1e-4)); //fails
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
