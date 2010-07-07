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
#include <boost/format.hpp>
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
Matrix Matrix_(size_t m, size_t n, const double* const data);

/**
 *  constructor with size and vector data, column order !!!
 */
Matrix Matrix_(size_t m, size_t n, const Vector& v);

/**
 *  constructor from Vector yielding v.size()*1 vector
 */
inline Matrix Matrix_(const Vector& v) { return Matrix_(v.size(),1,v);}

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
 * equals with an tolerance, prints out message if unequal
 */
bool assert_equal(const std::list<Matrix>& As, const std::list<Matrix>& Bs, double tol = 1e-9);

/**
 * check whether the rows of two matrices are linear indepdent
 */
bool linear_dependent(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * overload * for matrix-vector multiplication (as BOOST does not)
 */
inline Vector operator*(const Matrix& A, const Vector & v) { return prod(A,v);}

/**
 * BLAS Level-2 style e <- e + alpha*A*x
 */
void multiplyAdd(double alpha, const Matrix& A, const Vector& x, Vector& e);

/**
 * BLAS Level-2 style e <- e + A*x
 */
void multiplyAdd(const Matrix& A, const Vector& x, Vector& e);

/**
 * overload ^ for trans(A)*v
 * We transpose the vectors for speed.
 */
Vector operator^(const Matrix& A, const Vector & v);

/**
 * BLAS Level-2 style x <- x + alpha*A'*e
 */
void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, Vector& x);

/**
 * BLAS Level-2 style x <- x + A'*e
 */
void transposeMultiplyAdd(const Matrix& A, const Vector& e, Vector& x);

/**
 * BLAS Level-2 style x <- x + alpha*A'*e
 */
void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, SubVector x);

/**
 * overload * for vector*matrix multiplication (as BOOST does not)
 */
inline Vector operator*(const Vector & v, const Matrix& A) { return prod(v,A);}

/**
 * overload * for matrix multiplication (as BOOST does not)
 */
inline Matrix operator*(const Matrix& A, const Matrix& B) { return prod(A,B);}

/**
 * convert to column vector, column order !!!
 */
Vector Vector_(const Matrix& A);

/**
 * print a matrix
 */
void print(const Matrix& A, const std::string& s = "", std::ostream& stream = std::cout);

/**
 * save a matrix to file, which can be loaded by matlab
 */
void save(const Matrix& A, const std::string &s, const std::string& filename);

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
 * insert a submatrix IN PLACE at a specified location in a larger matrix
 * NOTE: there is no size checking
 * @param large matrix to be updated
 * @param small matrix to be inserted
 * @param i is the row of the upper left corner insert location
 * @param j is the column of the upper left corner insert location
 */
void insertSub(Matrix& big, const Matrix& small, size_t i, size_t j);

/**
 * extracts a column from a matrix
 * NOTE: using this without the underscore is the ublas version!
 * @param matrix to extract column from
 * @param index of the column
 * @return the column in vector form
 */
Vector column_(const Matrix& A, size_t j);

/**
 * inserts a column into a matrix IN PLACE
 * NOTE: there is no size checking
 * Alternate form allows for vectors smaller than the whole column to be inserted
 * @param A matrix to be modified in place
 * @param col is the vector to be inserted
 * @param j is the index to insert the column
 */
void insertColumn(Matrix& A, const Vector& col, size_t j);
void insertColumn(Matrix& A, const Vector& col, size_t i, size_t j);

/**
 * extracts a row from a matrix
 * @param matrix to extract row from
 * @param index of the row
 * @return the row in vector form
 */
Vector row_(const Matrix& A, size_t j);

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
 * Householder tranformation, zeros below diagonal
 * @param k number of columns to zero out below diagonal
 * @return nothing: in place !!!
 */
#ifdef GT_USE_LAPACK
#ifdef YA_BLAS
void householder(Matrix &A);
#endif
#endif

/**
 * backSubstitute U*x=b
 * @param U an upper triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of U*x=b
 * TODO: use boost
 */
Vector backSubstituteUpper(const Matrix& U, const Vector& b, bool unit=false);

/**
 * backSubstitute x'*U=b'
 * @param U an upper triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of x'*U=b'
 * TODO: use boost
 */
Vector backSubstituteUpper(const Vector& b, const Matrix& U, bool unit=false);

/**
 * backSubstitute L*x=b
 * @param L an lower triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of L*x=b
 * TODO: use boost
 */ 
Vector backSubstituteLower(const Matrix& L, const Vector& d, bool unit=false);

/**
 * create a matrix by stacking other matrices
 * Given a set of matrices: A1, A2, A3...
 * @return combined matrix [A1; A2; A3]
 */
Matrix stack(size_t nrMatrices, ...);

/**
 * create a matrix by concatenating
 * Given a set of matrices: A1, A2, A3...
 * If all matrices have the same size, specifying single matrix dimensions
 * will avoid the lookup of dimensions
 * @param matrices is a vector of matrices in the order to be collected
 * @param m is the number of rows of a single matrix
 * @param n is the number of columns of a single matrix
 * @return combined matrix [A1 A2 A3]
 */
Matrix collect(const std::vector<const Matrix *>& matrices, size_t m = 0, size_t n = 0);
Matrix collect(size_t nrMatrices, ...);

/**
 * scales a matrix row or column by the values in a vector
 * Arguments (Matrix, Vector) scales the columns,
 * (Vector, Matrix) scales the rows
 */
void vector_scale_inplace(const Vector& v, Matrix& A); // row
Matrix vector_scale(const Vector& v, const Matrix& A); // row
Matrix vector_scale(const Matrix& A, const Vector& v); // column

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
 * @param U output argument: rotation matrix
 * @param S output argument: vector of singular values, sorted by default, pass false as last argument to avoid sorting!!!
 * @param V output argument: rotation matrix
 * @param sort boolean flag to sort singular values and V
 * if m > n then U*S*V' = (m*n)*(n*n)*(n*n) (the m-n columns of U are of no use)
 * if m < n then U*S*V' = (m*m)*(m*m)*(m*n) (the n-m columns of V are of no use)
 */ 
void svd(const Matrix& A, Matrix& U, Vector& S, Matrix& V, bool sort=true);

/*
 * In place SVD, will throw an exception when m < n
 * @param A an m*n matrix is modified to contain U
 * @param S output argument: vector of singular values, sorted by default, pass false as last argument to avoid sorting!!!
 * @param V output argument: rotation matrix
 * @param sort boolean flag to sort singular values and V
 * if m > n then U*S*V' = (m*n)*(n*n)*(n*n) (the m-n columns of U are of no use)
 */
void svd(Matrix& A, Vector& S, Matrix& V, bool sort=true);

/** Use SVD to calculate inverse square root of a matrix */
Matrix inverse_square_root(const Matrix& A);

/** Use SVD to calculate the positive square root of a matrix */
Matrix square_root_positive(const Matrix& A);

/** Calculate the LL^t decomposition of a S.P.D matrix */
Matrix LLt(const Matrix& A);

/** Calculate the R^tR decomposition of a S.P.D matrix */
Matrix RtR(const Matrix& A);

/** Return the inverse of a S.P.D. matrix.  Inversion is done via Cholesky decomposition. */
Matrix cholesky_inverse(const Matrix &A);

/** Solve Ax=b with S.P.D. matrix using Davis' LDL code */
Vector solve_ldl(const Matrix& A, const Vector& b);

/**
 * Numerical exponential map, naive approach, not industrial strength !!!
 * @param A matrix to exponentiate
 * @param K number of iterations
 */
Matrix expm(const Matrix& A, int K=7);

// macro for unit tests
#define EQUALITY(expected,actual)\
  { if (!assert_equal(expected,actual)) \
    result_.addFailure(Failure(name_, __FILE__, __LINE__, #expected, #actual)); }

} // namespace gtsam
