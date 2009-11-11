/**
 * @file    Matrix.h
 * @brief   typedef and functions to augment Boost's ublas::matrix<double>
 * @author  Christian Potthast
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/tuple/tuple.hpp>
#include "Vector.h"

/**
 * Vector is a *global* typedef
 * wrap-matlab does this typedef as well
 * we use the default < double,row_major,unbounded_array<double> >
 */

#if ! defined (MEX_H)
typedef boost::numeric::ublas::matrix<double> Matrix;
#endif

namespace gtsam {

/**
 *  constructor with size and initial data, row order !
 */
Matrix Matrix_( size_t m, size_t n, const double* const data);

/**
 *  constructor with size and vector data, column order !!!
 */
Matrix Matrix_( size_t m, size_t n, const Vector& v);

/**
 *  nice constructor, dangerous as number of arguments must be exactly right
 *  and you have to pass doubles !!! always use 0.0 never 0
*/
Matrix Matrix_(size_t m, size_t n, ...);

/**
 * MATLAB like constructors
 */
Matrix zeros(size_t m, size_t n);
Matrix eye(size_t m, size_t n);
inline Matrix eye( size_t m ) { return eye(m,m); }
Matrix diag(const Vector& v);

/**
 * equals with an tolerance
 */
bool equal_with_abs_tol(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * equality is just equal_with_abs_tol 1e-9
 */
inline bool operator==(const Matrix& A, const Matrix& B) {
  return equal_with_abs_tol(A,B,1e-9);
}

/**
 * inequality 
 */
inline bool operator!=(const Matrix& A, const Matrix& B) {
  return !(A==B);
 }

/**
 * equals with an tolerance, prints out message if unequal
 */
bool assert_equal(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * overload * for matrix-vector multiplication (as BOOST does not)
 */
inline Vector operator*(const Matrix& A, const Vector & v) {
  if (A.size2()!=v.size()) throw(std::invalid_argument("Matrix operator* : A.n!=v.size"));
  return Vector(prod(A,v));
}

/**
 * overload * for matrix multiplication (as BOOST does not)
 */
inline Matrix operator*(const Matrix& A, const Matrix& B) {
  if (A.size2()!=B.size1()) throw(std::invalid_argument("Matrix operator* : A.n!=B.m"));
  return prod(A,B);
}

/**
 * convert to column vector, column order !!!
 */
Vector Vector_(const Matrix& A);

/**
 * print a matrix
 */
void print(const Matrix& A, const std::string& s = "");

/**
 * extract submatrix, slice semantics, i.e. range = [i1,i2[ excluding i2
 * @param A matrix
 * @param i1 first row index
 * @param i2 last  row index + 1
 * @param j1 first col index
 * @param j2 last  col index + 1
 * @return submatrix A(i1:i2-1,j1:j2-1)
 */
Matrix sub(const Matrix& A, size_t i1, size_t i2, size_t j1, size_t j2);

/**
 * extracts a column from a matrix
 * @param matrix to extract column from
 * @param index of the column
 * @return the column in vector form
 */
Vector column(const Matrix& A, size_t j);

/**
 * extracts a row from a matrix
 * @param matrix to extract row from
 * @param index of the row
 * @return the row in vector form
 */
Vector row(const Matrix& A, size_t j);

// left multiply with scalar
//inline Matrix operator*(double s, const Matrix& A) { return A*s;}

/**
 * solve AX=B via in-place Lu factorization and backsubstitution
 * After calling, A contains LU, B the solved RHS vectors
 */
void solve(Matrix& A, Matrix& B);

/**
 * invert A
 */
Matrix inverse(const Matrix& A);

/**
 * QR factorization, inefficient, best use imperative householder below
 * m*n matrix -> m*m Q, m*n R
 * @param A a matrix
 * @return <Q,R> rotation matrix Q, upper triangular R
 */ 
std::pair<Matrix,Matrix> qr(const Matrix& A);

/**
 * Imperative version of Householder rank 1 update
 */
void householder_update(Matrix &A, int j, double beta, const Vector& vjm);

/**
 * Imperative algorithm for in-place full elimination with
 * weights and constraint handling
 * @param A is a matrix to eliminate
 * @param b is the rhs
 * @param sigmas is a vector of the measurement standard deviation
 * @return list of r vectors, d  and sigma
 */
std::list<boost::tuple<Vector, double, double> >
weighted_eliminate(Matrix& A, Vector& b, const Vector& sigmas);

/**
 * Householder tranformation, Householder vectors below diagonal
 * @param k number of columns to zero out below diagonal
 * @param A matrix
 * @return nothing: in place !!!
 */
void householder_(Matrix& A, size_t k);

/**
 * Householder tranformation, zeros below diagonal
 * @param k number of columns to zero out below diagonal
 * @param A matrix
 * @return nothing: in place !!!
 */
void householder(Matrix& A, size_t k);

/**
 * backsubstitution
 * @param R an upper triangular matrix
 * @param b a RHS vector
 * @return the solution of Rx=b
 */ 
Vector backsubstitution(const Matrix& R, const Vector& b);

/**
 * create a matrix by stacking other matrices
 * Given a set of matrices: A1, A2, A3...
 * @return combined matrix [A1; A2; A3]
 */
Matrix stack(size_t nrMatrices, ...);

/**
 * create a matrix by concatenating
 * Given a set of matrices: A1, A2, A3...
 * @return combined matrix [A1 A2 A3]
 */
Matrix collect(std::vector<const Matrix *> matrices);
Matrix collect(size_t nrMatrices, ...);

/**
 * scales a matrix row or column by the values in a vector
 * Arguments (Matrix, Vector) scales the rows,
 * (Vector, Matrix) scales the columns
 */
Matrix vector_scale(const Matrix& A, const Vector& v); // row
Matrix vector_scale(const Vector& v, const Matrix& A); // column

/**
 * skew symmetric matrix returns this:
 *   0  -wz   wy
 *  wz    0  -wx
 * -wy   wx    0
 * @param wx 3 dimensional vector
 * @param wy
 * @param wz
 * @return a 3*3 skew symmetric matrix
*/
Matrix skewSymmetric(double wx, double wy, double wz);
inline Matrix skewSymmetric(const Vector& w) { return skewSymmetric(w(0),w(1),w(2));}

/**
 * SVD computes economy SVD A=U*S*V'
 * @param A an m*n matrix
 * @param U output argument: m*n matrix
 * @param S output argument: n-dim vector of singular values, *not* sorted !!!
 * @param V output argument: n*n matrix
 */ 
void svd(const Matrix& A, Matrix& U, Vector& S, Matrix& V);

// in-place version
void svd(Matrix& A, Vector& S, Matrix& V);

// macro for unit tests
#define EQUALITY(expected,actual)\
  { if (!assert_equal(expected,actual)) \
    result_.addFailure(Failure(name_, __FILE__, __LINE__, #expected, #actual)); }

} // namespace gtsam
