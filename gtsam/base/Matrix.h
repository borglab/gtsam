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
#include <gtsam/base/Vector.h>
#include <gtsam/config.h>      // Configuration from CMake

#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <boost/format.hpp>
#include <boost/tuple/tuple.hpp>


/**
 * Matrix is a typedef in the gtsam namespace
 * TODO: make a version to work with matlab wrapping
 * we use the default < double,col_major,unbounded_array<double> >
 */
namespace gtsam {

typedef Eigen::MatrixXd Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRowMajor;

// Create handy typedefs and constants for square-size matrices
// MatrixMN, MatrixN = MatrixNN, I_NxN, and Z_NxN, for M,N=1..9
#define GTSAM_MAKE_TYPEDEFS(SIZE, SUFFIX)   \
typedef Eigen::Matrix<double, SIZE, SIZE> Matrix##SUFFIX;  \
typedef Eigen::Matrix<double, 1, SIZE> Matrix1##SUFFIX;  \
typedef Eigen::Matrix<double, 2, SIZE> Matrix2##SUFFIX;  \
typedef Eigen::Matrix<double, 3, SIZE> Matrix3##SUFFIX;  \
typedef Eigen::Matrix<double, 4, SIZE> Matrix4##SUFFIX;  \
typedef Eigen::Matrix<double, 5, SIZE> Matrix5##SUFFIX;  \
typedef Eigen::Matrix<double, 6, SIZE> Matrix6##SUFFIX;  \
typedef Eigen::Matrix<double, 7, SIZE> Matrix7##SUFFIX;  \
typedef Eigen::Matrix<double, 8, SIZE> Matrix8##SUFFIX;  \
typedef Eigen::Matrix<double, 9, SIZE> Matrix9##SUFFIX;  \
static const Eigen::MatrixBase<Matrix##SUFFIX>::IdentityReturnType I_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Identity(); \
static const Eigen::MatrixBase<Matrix##SUFFIX>::ConstantReturnType Z_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Zero();

GTSAM_MAKE_TYPEDEFS(1,1);
GTSAM_MAKE_TYPEDEFS(2,2);
GTSAM_MAKE_TYPEDEFS(3,3);
GTSAM_MAKE_TYPEDEFS(4,4);
GTSAM_MAKE_TYPEDEFS(5,5);
GTSAM_MAKE_TYPEDEFS(6,6);
GTSAM_MAKE_TYPEDEFS(7,7);
GTSAM_MAKE_TYPEDEFS(8,8);
GTSAM_MAKE_TYPEDEFS(9,9);

// Matrix expressions for accessing parts of matrices
typedef Eigen::Block<Matrix> SubMatrix;
typedef Eigen::Block<const Matrix> ConstSubMatrix;

// Matlab-like syntax

/**
 * Creates an zeros matrix, with matlab-like syntax
 *
 * Note: if assigning a block (created from an Eigen block() function) of a matrix to zeros,
 * don't use this function, instead use ".setZero(m,n)" to avoid an Eigen error.
 */
GTSAM_EXPORT Matrix zeros(size_t m, size_t n);

/**
 * Creates an ones matrix, with matlab-like syntax
 */
GTSAM_EXPORT Matrix ones(size_t m, size_t n);

/**
 * Creates an identity matrix, with matlab-like syntax
 *
 * Note: if assigning a block (created from an Eigen block() function) of a matrix to identity,
 * don't use this function, instead use ".setIdentity(m,n)" to avoid an Eigen error.
 */
GTSAM_EXPORT Matrix eye(size_t m, size_t n);

/**
 * Creates a square identity matrix, with matlab-like syntax
 *
 * Note: if assigning a block (created from an Eigen block() function) of a matrix to identity,
 * don't use this function, instead use ".setIdentity(m)" to avoid an Eigen error.
 */
inline Matrix eye( size_t m ) { return eye(m,m); }
GTSAM_EXPORT Matrix diag(const Vector& v);

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
      if(boost::math::isnan(A(i,j)) ^ boost::math::isnan(B(i,j)))
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
GTSAM_EXPORT bool assert_equal(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * inequals with an tolerance, prints out message if within tolerance
 */
GTSAM_EXPORT bool assert_inequal(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * equals with an tolerance, prints out message if unequal
 */
GTSAM_EXPORT bool assert_equal(const std::list<Matrix>& As, const std::list<Matrix>& Bs, double tol = 1e-9);

/**
 * check whether the rows of two matrices are linear independent
 */
GTSAM_EXPORT bool linear_independent(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * check whether the rows of two matrices are linear dependent
 */
GTSAM_EXPORT bool linear_dependent(const Matrix& A, const Matrix& B, double tol = 1e-9);

/**
 * BLAS Level-2 style e <- e + alpha*A*x
 */
GTSAM_EXPORT void multiplyAdd(double alpha, const Matrix& A, const Vector& x, Vector& e);

/**
 * BLAS Level-2 style e <- e + A*x
 */
GTSAM_EXPORT void multiplyAdd(const Matrix& A, const Vector& x, Vector& e);

/**
 * overload ^ for trans(A)*v
 * We transpose the vectors for speed.
 */
GTSAM_EXPORT Vector operator^(const Matrix& A, const Vector & v);

/**
 * BLAS Level-2 style x <- x + alpha*A'*e
 */
GTSAM_EXPORT void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, Vector& x);

/**
 * BLAS Level-2 style x <- x + A'*e
 */
GTSAM_EXPORT void transposeMultiplyAdd(const Matrix& A, const Vector& e, Vector& x);

/**
 * BLAS Level-2 style x <- x + alpha*A'*e
 */
GTSAM_EXPORT void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, SubVector x);

/** products using old-style format to improve compatibility */
template<class MATRIX>
inline MATRIX prod(const MATRIX& A, const MATRIX&B) {
  MATRIX result = A * B;
  return result;
}

/**
 * print without optional string, must specify cout yourself
 */
GTSAM_EXPORT void print(const Matrix& A, const std::string& s, std::ostream& stream);

/**
 * print with optional string to cout
 */
GTSAM_EXPORT void print(const Matrix& A, const std::string& s = "");

/**
 * save a matrix to file, which can be loaded by matlab
 */
GTSAM_EXPORT void save(const Matrix& A, const std::string &s, const std::string& filename);

/**
 * Read a matrix from an input stream, such as a file.  Entries can be either
 * tab-, space-, or comma-separated, similar to the format read by the MATLAB
 * dlmread command.
 */
GTSAM_EXPORT std::istream& operator>>(std::istream& inputStream, Matrix& destinationMatrix);

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
 * @param fullMatrix matrix to be updated
 * @param subMatrix matrix to be inserted
 * @param i is the row of the upper left corner insert location
 * @param j is the column of the upper left corner insert location
 */
template <typename Derived1, typename Derived2>
void insertSub(Eigen::MatrixBase<Derived1>& fullMatrix, const Eigen::MatrixBase<Derived2>& subMatrix, size_t i, size_t j) {
  fullMatrix.block(i, j, subMatrix.rows(), subMatrix.cols()) = subMatrix;
}

/**
 * Create a matrix with submatrices along its diagonal
 */
GTSAM_EXPORT Matrix diag(const std::vector<Matrix>& Hs);

/**
 * Extracts a column view from a matrix that avoids a copy
 * @param A matrix to extract column from
 * @param j index of the column
 * @return a const view of the matrix
 */
template<class MATRIX>
const typename MATRIX::ConstColXpr column(const MATRIX& A, size_t j) {
  return A.col(j);
}

/**
 * Extracts a row view from a matrix that avoids a copy
 * @param A matrix to extract row from
 * @param j index of the row
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
GTSAM_EXPORT void insertColumn(Matrix& A, const Vector& col, size_t j);
GTSAM_EXPORT void insertColumn(Matrix& A, const Vector& col, size_t i, size_t j);

GTSAM_EXPORT Vector columnNormSquare(const Matrix &A);

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

/// Reshape functor
template <int OutM, int OutN, int OutOptions, int InM, int InN, int InOptions>
struct Reshape {
  //TODO replace this with Eigen's reshape function as soon as available. (There is a PR already pending : https://bitbucket.org/eigen/eigen/pull-request/41/reshape/diff)
  typedef Eigen::Map<const Eigen::Matrix<double, OutM, OutN, OutOptions> > ReshapedType;
  static inline ReshapedType reshape(const Eigen::Matrix<double, InM, InN, InOptions> & in) {
    return in.data();
  }
};

/// Reshape specialization that does nothing as shape stays the same (needed to not be ambiguous for square input equals square output)
template <int M, int InOptions>
struct Reshape<M, M, InOptions, M, M, InOptions> {
  typedef const Eigen::Matrix<double, M, M, InOptions> & ReshapedType;
  static inline ReshapedType reshape(const Eigen::Matrix<double, M, M, InOptions> & in) {
    return in;
  }
};

/// Reshape specialization that does nothing as shape stays the same
template <int M, int N, int InOptions>
struct Reshape<M, N, InOptions, M, N, InOptions> {
  typedef const Eigen::Matrix<double, M, N, InOptions> & ReshapedType;
  static inline ReshapedType reshape(const Eigen::Matrix<double, M, N, InOptions> & in) {
    return in;
  }
};

/// Reshape specialization that does transpose
template <int M, int N, int InOptions>
struct Reshape<N, M, InOptions, M, N, InOptions> {
  typedef typename Eigen::Matrix<double, M, N, InOptions>::ConstTransposeReturnType ReshapedType;
  static inline ReshapedType reshape(const Eigen::Matrix<double, M, N, InOptions> & in) {
    return in.transpose();
  }
};

template <int OutM, int OutN, int OutOptions, int InM, int InN, int InOptions>
inline typename Reshape<OutM, OutN, OutOptions, InM, InN, InOptions>::ReshapedType reshape(const Eigen::Matrix<double, InM, InN, InOptions> & m){
  BOOST_STATIC_ASSERT(InM * InN == OutM * OutN);
  return Reshape<OutM, OutN, OutOptions, InM, InN, InOptions>::reshape(m);
}

/**
 * solve AX=B via in-place Lu factorization and backsubstitution
 * After calling, A contains LU, B the solved RHS vectors
 */
GTSAM_EXPORT void solve(Matrix& A, Matrix& B);

/**
 * invert A
 */
GTSAM_EXPORT Matrix inverse(const Matrix& A);

/**
 * QR factorization, inefficient, best use imperative householder below
 * m*n matrix -> m*m Q, m*n R
 * @param A a matrix
 * @return <Q,R> rotation matrix Q, upper triangular R
 */ 
GTSAM_EXPORT std::pair<Matrix,Matrix> qr(const Matrix& A);

/**
 * QR factorization using Eigen's internal block QR algorithm
 * @param A is the input matrix, and is the output
 * @param clear_below_diagonal enables zeroing out below diagonal
 */
void inplace_QR(Matrix& A);

/**
 * Imperative algorithm for in-place full elimination with
 * weights and constraint handling
 * @param A is a matrix to eliminate
 * @param b is the rhs
 * @param sigmas is a vector of the measurement standard deviation
 * @return list of r vectors, d  and sigma
 */
GTSAM_EXPORT std::list<boost::tuple<Vector, double, double> >
weighted_eliminate(Matrix& A, Vector& b, const Vector& sigmas);

/**
 * Householder transformation, Householder vectors below diagonal
 * @param k number of columns to zero out below diagonal
 * @param A matrix
 * @param copy_vectors - true to copy Householder vectors below diagonal
 * @return nothing: in place !!!
 */
GTSAM_EXPORT void householder_(Matrix& A, size_t k, bool copy_vectors=true);

/**
 * Householder tranformation, zeros below diagonal
 * @param k number of columns to zero out below diagonal
 * @param A matrix
 * @return nothing: in place !!!
 */
GTSAM_EXPORT void householder(Matrix& A, size_t k);

/**
 * backSubstitute U*x=b
 * @param U an upper triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of U*x=b
 */
GTSAM_EXPORT Vector backSubstituteUpper(const Matrix& U, const Vector& b, bool unit=false);

/**
 * backSubstitute x'*U=b'
 * @param U an upper triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of x'*U=b'
 */
//TODO: is this function necessary? it isn't used
GTSAM_EXPORT Vector backSubstituteUpper(const Vector& b, const Matrix& U, bool unit=false);

/**
 * backSubstitute L*x=b
 * @param L an lower triangular matrix
 * @param b an RHS vector
 * @param unit, set true if unit triangular
 * @return the solution x of L*x=b
 */ 
GTSAM_EXPORT Vector backSubstituteLower(const Matrix& L, const Vector& b, bool unit=false);

/**
 * create a matrix by stacking other matrices
 * Given a set of matrices: A1, A2, A3...
 * @param ... pointers to matrices to be stacked
 * @return combined matrix [A1; A2; A3]
 */
GTSAM_EXPORT Matrix stack(size_t nrMatrices, ...);
GTSAM_EXPORT Matrix stack(const std::vector<Matrix>& blocks);

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
GTSAM_EXPORT Matrix collect(const std::vector<const Matrix *>& matrices, size_t m = 0, size_t n = 0);
GTSAM_EXPORT Matrix collect(size_t nrMatrices, ...);

/**
 * scales a matrix row or column by the values in a vector
 * Arguments (Matrix, Vector) scales the columns,
 * (Vector, Matrix) scales the rows
 * @param inf_mask when true, will not scale with a NaN or inf value.
 */
GTSAM_EXPORT void vector_scale_inplace(const Vector& v, Matrix& A, bool inf_mask = false); // row
GTSAM_EXPORT Matrix vector_scale(const Vector& v, const Matrix& A, bool inf_mask = false); // row
GTSAM_EXPORT Matrix vector_scale(const Matrix& A, const Vector& v, bool inf_mask = false); // column

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
GTSAM_EXPORT Matrix3 skewSymmetric(double wx, double wy, double wz);
template<class Derived>
inline Matrix3 skewSymmetric(const Eigen::MatrixBase<Derived>& w) { return skewSymmetric(w(0),w(1),w(2));}

/** Use Cholesky to calculate inverse square root of a matrix */
GTSAM_EXPORT Matrix inverse_square_root(const Matrix& A);

/** Calculate the LL^t decomposition of a S.P.D matrix */
GTSAM_EXPORT Matrix LLt(const Matrix& A);

/** Calculate the R^tR decomposition of a S.P.D matrix */
GTSAM_EXPORT Matrix RtR(const Matrix& A);

/** Return the inverse of a S.P.D. matrix.  Inversion is done via Cholesky decomposition. */
GTSAM_EXPORT Matrix cholesky_inverse(const Matrix &A);

/**
 * SVD computes economy SVD A=U*S*V'
 * @param A an m*n matrix
 * @param U output argument: rotation matrix
 * @param S output argument: sorted vector of singular values
 * @param V output argument: rotation matrix
 * if m > n then U*S*V' = (m*n)*(n*n)*(n*n)
 * if m < n then U*S*V' = (m*m)*(m*m)*(m*n)
 * Careful! The dimensions above reflect V', not V, which is n*m if m<n.
 * U is a basis in R^m, V is a basis in R^n
 * You can just pass empty matrices U,V, and vector S, they will be re-allocated.
 */
GTSAM_EXPORT void svd(const Matrix& A, Matrix& U, Vector& S, Matrix& V);

/**
 * Direct linear transform algorithm that calls svd
 * to find a vector v that minimizes the algebraic error A*v
 * @param A of size m*n, where m>=n (pad with zero rows if not!)
 * Returns rank of A, minimum error (singular value),
 * and corresponding eigenvector (column of V, with A=U*S*V')
 */
GTSAM_EXPORT boost::tuple<int, double, Vector>
DLT(const Matrix& A, double rank_tol = 1e-9);

/**
 * Numerical exponential map, naive approach, not industrial strength !!!
 * @param A matrix to exponentiate
 * @param K number of iterations
 */
GTSAM_EXPORT Matrix expm(const Matrix& A, size_t K=7);

std::string formatMatrixIndented(const std::string& label, const Matrix& matrix, bool makeVectorHorizontal = false);

} // namespace gtsam

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
  namespace serialization {

    // split version - sends sizes ahead
    template<class Archive>
    void save(Archive & ar, const gtsam::Matrix & m, unsigned int /*version*/) {
      const size_t rows = m.rows(), cols = m.cols();
      ar << BOOST_SERIALIZATION_NVP(rows);
      ar << BOOST_SERIALIZATION_NVP(cols);
      ar << make_nvp("data", make_array(m.data(), m.size()));
    }

    template<class Archive>
    void load(Archive & ar, gtsam::Matrix & m, unsigned int /*version*/) {
      size_t rows, cols;
      ar >> BOOST_SERIALIZATION_NVP(rows);
      ar >> BOOST_SERIALIZATION_NVP(cols);
      m.resize(rows, cols);
      ar >> make_nvp("data", make_array(m.data(), m.size()));
    }

  } // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(gtsam::Matrix);

