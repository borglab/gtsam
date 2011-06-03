/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Matrix.h
 * @brief   typedef and functions to augment Eigen's MatrixXd
 * @author  Christian Potthast
 * @author  Kai Ni
 * @author  Frank Dellaert
 * @author  Alex Cunningham
 */

// \callgraph

#pragma once

#include <list>
#include <boost/format.hpp>
#include <boost/tuple/tuple.hpp>
#include <gtsam/3rdparty/Eigen/QR>
#include <gtsam/base/Vector.h>

/**
 * Matrix is a *global* typedef
 * we use the default < double,row_major,unbounded_array<double> >
 */

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixColMajor;

// Matrix expressions for accessing parts of matrices
typedef Eigen::Block<Matrix> SubMatrix;
typedef Eigen::Block<const Matrix> ConstSubMatrix;

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
template <class MATRIX>
bool equal_with_abs_tol(const Eigen::DenseBase<MATRIX>& A, const Eigen::DenseBase<MATRIX>& B, double tol = 1e-9) {

	const size_t n1 = A.cols(), m1 = A.rows();
	const size_t n2 = B.cols(), m2 = B.rows();

	if(m1!=m2 || n1!=n2) return false;

	for(size_t i=0; i<m1; i++)
		for(size_t j=0; j<n1; j++) {
			if(std::isnan(A(i,j)) xor std::isnan(B(i,j)))
				return false;
			else if(fabs(A(i,j) - B(i,j)) > tol)
				return false;
		}
	return true;
}

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
// FIXME: make better use of templates to test these properly
bool assert_equal(const Matrix& A, const Matrix& B, double tol = 1e-9);
bool assert_equal(const MatrixColMajor& A, const MatrixColMajor& B, double tol = 1e-9);

/**
 * equals with an tolerance, prints out message if unequal
 */
bool assert_equal(const std::list<Matrix>& As, const std::list<Matrix>& Bs, double tol = 1e-9);

/**
 * check whether the rows of two matrices are linear independent
 */
bool linear_independent(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * check whether the rows of two matrices are linear dependent
 */
bool linear_dependent(const Matrix& A, const Matrix& B, double tol = 1e-9);

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

/** products using old-style format to improve compatibility */
template<class MATRIX>
inline MATRIX prod(const MATRIX& A, const MATRIX&B) {
	MATRIX result = A * B;
	return result;
}

/**
 * convert to column vector, column order !!!
 */
Vector Vector_(const Matrix& A);

/**
 * print a matrix
 */
void print(const Matrix& A, const std::string& s = "", std::ostream& stream = std::cout);
void print(const MatrixColMajor& A, const std::string& s = "", std::ostream& stream = std::cout);

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
template<class MATRIX>
Eigen::Block<const MATRIX> sub(const MATRIX& A, size_t i1, size_t i2, size_t j1, size_t j2) {
	size_t m=i2-i1, n=j2-j1;
	return A.block(i1,j1,m,n);
}

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
 * Extracts a column view from a matrix that avoids a copy
 * @param matrix to extract column from
 * @param index of the column
 * @return a const view of the matrix
 */
template<class MATRIX>
const typename MATRIX::ConstColXpr column(const MATRIX& A, size_t j) {
	return A.col(j);
}

/**
 * Extracts a row view from a matrix that avoids a copy
 * @param matrix to extract row from
 * @param index of the row
 * @return a const view of the matrix
 */
template<class MATRIX>
const typename MATRIX::ConstRowXpr row(const MATRIX& A, size_t j) {
	return A.row(j);
}

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

Vector columnNormSquare(const MatrixColMajor &A);

/**
 * Zeros all of the elements below the diagonal of a matrix, in place
 * @param A is a matrix, to be modified in place
 * @param cols is the number of columns to zero, use zero for all columns
 */
template<class MATRIX>
void zeroBelowDiagonal(MATRIX& A, size_t cols=0) {
	const size_t m = A.rows(), n = A.cols();
	const size_t k = (cols) ? std::min(cols, std::min(m,n)) : std::min(m,n);
	for (size_t j=0; j<k; ++j)
		A.col(j).segment(j+1, m-(j+1)).setZero();
}

/**
 * static transpose function, just calls Eigen transpose member function
 */
inline Matrix trans(const Matrix& A) { return A.transpose(); }

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
 * QR factorization using Eigen's internal block QR algorithm
 * @param A is the input matrix, and is the output
 * @param clear_below_diagonal enables zeroing out below diagonal
 */
template <class MATRIX>
void inplace_QR(MATRIX& A, bool clear_below_diagonal=true) {
	size_t rows = A.rows();
	size_t cols = A.cols();
	size_t size = std::min(rows,cols);

	typedef Eigen::internal::plain_diag_type<MatrixColMajor>::type HCoeffsType;
	typedef Eigen::internal::plain_row_type<MatrixColMajor>::type RowVectorType;
	HCoeffsType hCoeffs(size);
	RowVectorType temp(cols);

	Eigen::internal::householder_qr_inplace_blocked(A, hCoeffs, 48, temp.data());

	zeroBelowDiagonal(A);
}

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
 * Householder tranformation, Householder vectors below diagonal
 * Column Major Version
 * @param k number of columns to zero out below diagonal
 * @param A matrix
 * @param copy_vectors - true to copy Householder vectors below diagonal
 * @return nothing: in place !!!
 */
void householder_(MatrixColMajor& A, size_t k, bool copy_vectors=true);

/**
 * Householder tranformation, zeros below diagonal
 * Column Major version
 * @param k number of columns to zero out below diagonal
 * @param A matrix
 * @return nothing: in place !!!
 */
void householder(MatrixColMajor& A, size_t k);

/**
 * backSubstitute U*x=b
 * @param U an upper triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of U*x=b
 */
//FIXME: add back expression form
//template<class MATRIX, class VECTOR>
//Vector backSubstituteUpper(const MATRIX& U, const VECTOR& b, bool unit=false);
Vector backSubstituteUpper(const Matrix& U, const Vector& b, bool unit=false);

/**
 * backSubstitute x'*U=b'
 * @param U an upper triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of x'*U=b'
 */
//FIXME: add back expression form
//TODO: is this function necessary? it isn't used
//template<class VECTOR, class MATRIX>
//Vector backSubstituteUpper(const VECTOR& b, const MATRIX& U, bool unit=false);
Vector backSubstituteUpper(const Vector& b, const Matrix& U, bool unit=false);

/**
 * backSubstitute L*x=b
 * @param L an lower triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of L*x=b
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
void vector_scale_inplace(const Vector& v, MatrixColMajor& A); // row
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

/** Use SVD to calculate inverse square root of a matrix */
Matrix inverse_square_root(const Matrix& A);

/** Calculate the LL^t decomposition of a S.P.D matrix */
Matrix LLt(const Matrix& A);

/** Calculate the R^tR decomposition of a S.P.D matrix */
Matrix RtR(const Matrix& A);

/** Return the inverse of a S.P.D. matrix.  Inversion is done via Cholesky decomposition. */
Matrix cholesky_inverse(const Matrix &A);

/**
 * SVD computes economy SVD A=U*S*V'
 * @param A an m*n matrix
 * @param U output argument: rotation matrix
 * @param S output argument: vector of singular values, sorted by default, pass false as last argument to avoid sorting!!!
 * @param V output argument: rotation matrix
 * if m > n then U*S*V' = (m*n)*(n*n)*(n*n) (the m-n columns of U are of no use)
 * if m < n then U*S*V' = (m*m)*(m*m)*(m*n) (the n-m columns of V are of no use)
 * Careful! The dimensions above reflect V', not V, which is n*m if m<n.
 * U is a basis in R^m, V is a basis in R^n
 * You can just pass empty matrices U,V, and vector S, they will be re-allocated.
 */
void svd(const Matrix& A, Matrix& U, Vector& S, Matrix& V);

/*
 * In place SVD, will throw an exception when m < n
 * @param A an m*n matrix is modified to contain U
 * @param S output argument: vector of singular values, sorted by default, pass false as last argument to avoid sorting!!!
 * @param V output argument: rotation matrix
 * if m > n then U*S*V' = (m*n)*(n*n)*(n*n) (the m-n columns of U are of no use)
 * You can just pass empty matrix V and vector S, they will be re-allocated.
 */
void svd(Matrix& A, Vector& S, Matrix& V);

/**
 * Direct linear transform algorithm that calls svd
 * to find a vector v that minimizes the algebraic error A*v
 * @param A of size m*n, where m>=n (pad with zero rows if not!)
 * Returns rank of A, minimum error (singular value),
 * and corresponding eigenvector (column of V, with A=U*S*V')
 */
boost::tuple<int, double, Vector>
DLT(const Matrix& A, double rank_tol = 1e-9);

/**
 * Numerical exponential map, naive approach, not industrial strength !!!
 * @param A matrix to exponentiate
 * @param K number of iterations
 */
Matrix expm(const Matrix& A, size_t K=7);

// macro for unit tests
#define EQUALITY(expected,actual)\
  { if (!assert_equal(expected,actual)) \
    result_.addFailure(Failure(name_, __FILE__, __LINE__, #expected, #actual)); }

} // namespace gtsam

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
namespace serialization {

// split version for Row-major matrix - sends sizes ahead
template<class Archive>
void save(Archive & ar, const Matrix & m, unsigned int version)
{
	const int rows = m.rows(), cols = m.cols(), elements = rows*cols;
	std::vector<double> raw_data(elements);
	std::copy(m.data(), m.data()+elements, raw_data.begin());
	ar << make_nvp("rows", rows);
	ar << make_nvp("cols", cols);
	ar << make_nvp("data", raw_data);
}
template<class Archive>
void load(Archive & ar, Matrix & m, unsigned int version)
{
	size_t rows, cols;
	std::vector<double> raw_data;
	ar >> make_nvp("rows", rows);
	ar >> make_nvp("cols", cols);
	ar >> make_nvp("data", raw_data);
	m = Matrix(rows, cols);
	std::copy(raw_data.begin(), raw_data.end(), m.data());
}

// split version for Column-major matrix - sends sizes ahead
template<class Archive>
void save(Archive & ar, const MatrixColMajor & m, unsigned int version)
{
	const int rows = m.rows(), cols = m.cols(), elements = rows*cols;
	std::vector<double> raw_data(elements);
	std::copy(m.data(), m.data()+elements, raw_data.begin());
	ar << make_nvp("rows", rows);
	ar << make_nvp("cols", cols);
	ar << make_nvp("data", raw_data);
}
template<class Archive>
void load(Archive & ar, MatrixColMajor & m, unsigned int version)
{
	size_t rows, cols;
	std::vector<double> raw_data;
	ar >> make_nvp("rows", rows);
	ar >> make_nvp("cols", cols);
	ar >> make_nvp("data", raw_data);
	m = MatrixColMajor(rows, cols);
	std::copy(raw_data.begin(), raw_data.end(), m.data());
}

} // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(Matrix)
BOOST_SERIALIZATION_SPLIT_FREE(MatrixColMajor)

