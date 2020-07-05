/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Matrix.cpp
 * @brief  matrix class
 * @author Christian Potthast
 */

#include <gtsam/global_includes.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/FastList.h>
#include <Eigen/SVD>
#include <Eigen/LU>

#include <boost/tuple/tuple.hpp>
#include <boost/tokenizer.hpp>

#include <cstdarg>
#include <cstring>
#include <iomanip>
#include <list>
#include <fstream>
#include <limits>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
bool assert_equal(const Matrix& expected, const Matrix& actual, double tol) {

  if (equal_with_abs_tol(expected,actual,tol)) return true;

  size_t n1 = expected.cols(), m1 = expected.rows();
  size_t n2 = actual.cols(), m2 = actual.rows();

  cout << "not equal:" << endl;
  print(expected,"expected = ");
  print(actual,"actual = ");
  if(m1!=m2 || n1!=n2)
    cout << m1 << "," << n1 << " != " << m2 << "," << n2 << endl;
  else {
    Matrix diff = actual-expected;
    print(diff, "actual - expected = ");
  }
  return false;
}

/* ************************************************************************* */
bool assert_inequal(const Matrix& A, const Matrix& B, double tol) {
  if (!equal_with_abs_tol(A,B,tol)) return true;
  cout << "Erroneously equal:" << endl;
  print(A, "A = ");
  print(B, "B = ");
  return false;
}

/* ************************************************************************* */
bool assert_equal(const std::list<Matrix>& As, const std::list<Matrix>& Bs, double tol) {
  if (As.size() != Bs.size()) return false;

  list<Matrix>::const_iterator itA, itB;
  itA = As.begin(); itB = Bs.begin();
  for (; itA != As.end(); itA++, itB++)
    if (!assert_equal(*itB, *itA, tol))
      return false;

  return true;
}

/* ************************************************************************* */
static bool is_linear_dependent(const Matrix& A, const Matrix& B, double tol) {
  // This local static function is used by linear_independent and
  // linear_dependent just below.
  size_t n1 = A.cols(), m1 = A.rows();
  size_t n2 = B.cols(), m2 = B.rows();

  bool dependent = true;
  if(m1!=m2 || n1!=n2) dependent = false;

  for(size_t i=0; dependent && i<m1; i++) {
    if (!gtsam::linear_dependent(Vector(row(A,i)), Vector(row(B,i)), tol))
      dependent = false;
  }

  return dependent;
}

/* ************************************************************************* */
bool linear_independent(const Matrix& A, const Matrix& B, double tol) {
  if(!is_linear_dependent(A, B, tol))
    return true;
  else {
    cout << "not linearly dependent:" << endl;
    print(A,"A = ");
    print(B,"B = ");
    if(A.rows()!=B.rows() || A.cols()!=B.cols())
      cout << A.rows() << "x" << A.cols() << " != " << B.rows() << "x" << B.cols() << endl;
    return false;
  }
}

/* ************************************************************************* */
bool linear_dependent(const Matrix& A, const Matrix& B, double tol) {
  if(is_linear_dependent(A, B, tol))
    return true;
  else {
    cout << "not linearly dependent:" << endl;
    print(A,"A = ");
    print(B,"B = ");
    if(A.rows()!=B.rows() || A.cols()!=B.cols())
      cout << A.rows() << "x" << A.cols() << " != " << B.rows() << "x" << B.cols() << endl;
    return false;
  }
}

/* ************************************************************************* */
Vector operator^(const Matrix& A, const Vector & v) {
  if (A.rows()!=v.size()) throw std::invalid_argument(
      boost::str(boost::format("Matrix operator^ : A.m(%d)!=v.size(%d)") % A.rows() % v.size()));
//  Vector vt = v.transpose();
//  Vector vtA = vt * A;
//  return vtA.transpose();
  return A.transpose() * v;
}

/* ************************************************************************* */
//3 argument call
void print(const Matrix& A, const string &s, ostream& stream) {
  static const Eigen::IOFormat matlab(
      Eigen::StreamPrecision, // precision
      0, // flags
      ", ", // coeffSeparator
      ";\n", // rowSeparator
      "\t",  // rowPrefix
      "", // rowSuffix
      "[\n", // matPrefix
      "\n]" // matSuffix
      );
  cout << s << A.format(matlab) << endl;
}

/* ************************************************************************* */
//1 or 2 argument call
void print(const Matrix& A, const string &s){
  print(A, s, cout);
}

/* ************************************************************************* */
void save(const Matrix& A, const string &s, const string& filename) {
  fstream stream(filename.c_str(), fstream::out | fstream::app);
  print(A, s + "=", stream);
  stream.close();
}

/* ************************************************************************* */
istream& operator>>(istream& inputStream, Matrix& destinationMatrix) {
  string line;
  FastList<vector<double> > coeffs;
  bool first = true;
  size_t width = 0;
  size_t height = 0;
  while(getline(inputStream, line)) {
    // Read coefficients from file
    coeffs.push_back(vector<double>());
    if(!first)
      coeffs.back().reserve(width);
    stringstream lineStream(line);
    std::copy(istream_iterator<double>(lineStream), istream_iterator<double>(),
      back_insert_iterator<vector<double> >(coeffs.back()));
    if(first)
      width = coeffs.back().size();
    if(coeffs.back().size() != width)
      throw runtime_error("Error reading matrix from input stream, inconsistent numbers of elements in rows");
    ++ height;
  }

  // Copy coefficients to matrix
  destinationMatrix.resize(height, width);
  int row = 0;
  for(const vector<double>& rowVec: coeffs) {
    destinationMatrix.row(row) = Eigen::Map<const Eigen::RowVectorXd>(&rowVec[0], width);
    ++ row;
  }

  return inputStream;
}

/* ************************************************************************* */
Matrix diag(const std::vector<Matrix>& Hs) {
  size_t rows = 0, cols = 0;
  for (size_t i = 0; i<Hs.size(); ++i) {
    rows+= Hs[i].rows();
    cols+= Hs[i].cols();
  }
  Matrix results = Matrix::Zero(rows,cols);
  size_t r = 0, c = 0;
  for (size_t i = 0; i<Hs.size(); ++i) {
    insertSub(results, Hs[i], r, c);
    r+=Hs[i].rows();
    c+=Hs[i].cols();
  }
  return results;
}

/* ************************************************************************* */
Vector columnNormSquare(const Matrix &A) {
  Vector v (A.cols()) ;
  for ( size_t i = 0 ; i < (size_t) A.cols() ; ++i ) {
    v[i] = A.col(i).dot(A.col(i));
  }
  return v ;
}

/* ************************************************************************* */
/** Householder QR factorization, Golub & Van Loan p 224, explicit version    */
/* ************************************************************************* */
pair<Matrix,Matrix> qr(const Matrix& A) {
  const size_t m = A.rows(), n = A.cols(), kprime = min(m,n);

  Matrix Q=Matrix::Identity(m,m),R(A);
  Vector v(m);

  // loop over the kprime first columns
  for(size_t j=0; j < kprime; j++){

    // we now work on the matrix (m-j)*(n-j) matrix A(j:end,j:end)
    const size_t mm=m-j;

    // copy column from matrix to xjm, i.e. x(j:m) = A(j:m,j)
    Vector xjm(mm);
    for(size_t k = 0 ; k < mm; k++)
      xjm(k) = R(j+k, j);

    // calculate the Householder vector v
    double beta; Vector vjm;
    boost::tie(beta,vjm) = house(xjm);

    // pad with zeros to get m-dimensional vector v
    for(size_t k = 0 ; k < m; k++)
      v(k) = k<j ? 0.0 : vjm(k-j);

    // create Householder reflection matrix Qj = I-beta*v*v'
    Matrix Qj = Matrix::Identity(m,m) - beta * v * v.transpose();

    R = Qj * R; // update R
    Q = Q * Qj; // update Q

  } // column j

  return make_pair(Q,R);
}

/* ************************************************************************* */
list<boost::tuple<Vector, double, double> >
weighted_eliminate(Matrix& A, Vector& b, const Vector& sigmas) {
  size_t m = A.rows(), n = A.cols(); // get size(A)
  size_t maxRank = min(m,n);

  // create list
  list<boost::tuple<Vector, double, double> > results;

  Vector pseudo(m); // allocate storage for pseudo-inverse
  Vector weights = sigmas.array().square().inverse(); // calculate weights once

  // We loop over all columns, because the columns that can be eliminated
  // are not necessarily contiguous. For each one, estimate the corresponding
  // scalar variable x as d-rS, with S the separator (remaining columns).
  // Then update A and b by substituting x with d-rS, zero-ing out x's column.
  for (size_t j=0; j<n; ++j) {
    // extract the first column of A
    Vector a(column(A, j));

    // Calculate weighted pseudo-inverse and corresponding precision
    double precision = weightedPseudoinverse(a, weights, pseudo);

    // if precision is zero, no information on this column
    if (precision < 1e-8) continue;

    // create solution and copy into r
    Vector r(Vector::Unit(n,j));
    for (size_t j2=j+1; j2<n; ++j2)
      r(j2) = pseudo.dot(A.col(j2));

    // create the rhs
    double d = pseudo.dot(b);

    // construct solution (r, d, sigma)
    // TODO: avoid sqrt, store precision or at least variance
    results.push_back(boost::make_tuple(r, d, 1./sqrt(precision)));

    // exit after rank exhausted
    if (results.size()>=maxRank) break;

    // update A, b, expensive, using outer product
    // A' \define A_{S}-a*r and b'\define b-d*a
    A -= a * r.transpose();
    b -= d * a;
  }

  return results;
}

/* ************************************************************************* */
/** Imperative version of Householder QR factorization, Golub & Van Loan p 224
 * version with Householder vectors below diagonal, as in GVL
 */

/* ************************************************************************* */
void householder_(Matrix& A, size_t k, bool copy_vectors) {
  const size_t m = A.rows(), n = A.cols(), kprime = min(k,min(m,n));
  // loop over the kprime first columns
  for(size_t j=0; j < kprime; j++) {
    // copy column from matrix to vjm, i.e. v(j:m) = A(j:m,j)
    Vector vjm = A.col(j).segment(j, m-j);

    // calculate the Householder vector, in place
    double beta = houseInPlace(vjm);

    // do outer product update A(j:m,:) = (I-beta vv')*A = A - v*(beta*A'*v)' = A - v*w'
    gttic(householder_update); // bottleneck for system
    // don't touch old columns
    Vector w = beta * A.block(j,j,m-j,n-j).transpose() * vjm;
    A.block(j,j,m-j,n-j) -= vjm * w.transpose();
    gttoc(householder_update);

    // the Householder vector is copied in the zeroed out part
    if (copy_vectors) {
      gttic(householder_vector_copy);
      A.col(j).segment(j+1, m-(j+1)) = vjm.segment(1, m-(j+1));
      gttoc(householder_vector_copy);
    }
  } // column j
}

/* ************************************************************************* */
void householder(Matrix& A, size_t k) {
  // version with zeros below diagonal
  gttic(householder_);
  householder_(A,k,false);
  gttoc(householder_);
//  gttic(householder_zero_fill);
//  const size_t m = A.rows(), n = A.cols(), kprime = min(k,min(m,n));
//  for(size_t j=0; j < kprime; j++)
//    A.col(j).segment(j+1, m-(j+1)).setZero();
//  gttoc(householder_zero_fill);
}

/* ************************************************************************* */
Vector backSubstituteLower(const Matrix& L, const Vector& b, bool unit) {
  // @return the solution x of L*x=b
  assert(L.rows() == L.cols());
  if (unit)
    return L.triangularView<Eigen::UnitLower>().solve(b);
  else
    return L.triangularView<Eigen::Lower>().solve(b);
}

/* ************************************************************************* */
Vector backSubstituteUpper(const Matrix& U, const Vector& b, bool unit) {
  // @return the solution x of U*x=b
  assert(U.rows() == U.cols());
  if (unit)
    return U.triangularView<Eigen::UnitUpper>().solve(b);
  else
    return U.triangularView<Eigen::Upper>().solve(b);
}

/* ************************************************************************* */
Vector backSubstituteUpper(const Vector& b, const Matrix& U, bool unit) {
  // @return the solution x of x'*U=b'
  assert(U.rows() == U.cols());
  if (unit)
    return U.triangularView<Eigen::UnitUpper>().transpose().solve<Eigen::OnTheLeft>(b);
  else
    return U.triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheLeft>(b);
}

/* ************************************************************************* */
Matrix stack(size_t nrMatrices, ...)
{
  size_t dimA1 = 0;
  size_t dimA2 = 0;
  va_list ap;
  va_start(ap, nrMatrices);
  for(size_t i = 0 ; i < nrMatrices ; i++) {
    Matrix *M = va_arg(ap, Matrix *);
    dimA1 += M->rows();
    dimA2 =  M->cols();  // TODO: should check if all the same !
  }
  va_end(ap);
  va_start(ap, nrMatrices);
  Matrix A(dimA1, dimA2);
  size_t vindex = 0;
  for( size_t i = 0 ; i < nrMatrices ; i++) {
    Matrix *M = va_arg(ap, Matrix *);
    for(size_t d1 = 0; d1 < (size_t) M->rows(); d1++)
      for(size_t d2 = 0; d2 < (size_t) M->cols(); d2++)
        A(vindex+d1, d2) = (*M)(d1, d2);
    vindex += M->rows();
  }

  return A;
}

/* ************************************************************************* */
Matrix stack(const std::vector<Matrix>& blocks) {
  if (blocks.size() == 1) return blocks.at(0);
  DenseIndex nrows = 0, ncols = blocks.at(0).cols();
  for(const Matrix& mat: blocks) {
    nrows += mat.rows();
    if (ncols != mat.cols())
      throw invalid_argument("Matrix::stack(): column size mismatch!");
  }
  Matrix result(nrows, ncols);

  DenseIndex cur_row = 0;
  for(const Matrix& mat: blocks) {
    result.middleRows(cur_row, mat.rows()) = mat;
    cur_row += mat.rows();
  }
  return result;
}

/* ************************************************************************* */
Matrix collect(const std::vector<const Matrix *>& matrices, size_t m, size_t n)
{
  // if we have known and constant dimensions, use them
  size_t dimA1 = m;
  size_t dimA2 = n*matrices.size();
  if (!m && !n) {
    for(const Matrix* M: matrices) {
      dimA1 =  M->rows();  // TODO: should check if all the same !
      dimA2 += M->cols();
    }
  }

  // stl::copy version
  Matrix A(dimA1, dimA2);
  size_t hindex = 0;
  for(const Matrix* M: matrices) {
    size_t row_len = M->cols();
    A.block(0, hindex, dimA1, row_len) = *M;
    hindex += row_len;
  }

  return A;
}

/* ************************************************************************* */
Matrix collect(size_t nrMatrices, ...)
{
  vector<const Matrix *> matrices;
  va_list ap;
  va_start(ap, nrMatrices);
  for( size_t i = 0 ; i < nrMatrices ; i++) {
    Matrix *M = va_arg(ap, Matrix *);
    matrices.push_back(M);
  }
  return collect(matrices);
}

/* ************************************************************************* */
// row scaling, in-place
void vector_scale_inplace(const Vector& v, Matrix& A, bool inf_mask) {
  const DenseIndex m = A.rows();
  if (inf_mask) {
    for (DenseIndex i=0; i<m; ++i) {
      const double& vi = v(i);
      if (std::isfinite(vi))
        A.row(i) *= vi;
    }
  } else {
    for (DenseIndex i=0; i<m; ++i)
      A.row(i) *= v(i);
  }
}

/* ************************************************************************* */
// row scaling
Matrix vector_scale(const Vector& v, const Matrix& A, bool inf_mask) {
  Matrix M(A);
  vector_scale_inplace(v, M, inf_mask);
  return M;
}

/* ************************************************************************* */
// column scaling
Matrix vector_scale(const Matrix& A, const Vector& v, bool inf_mask) {
  Matrix M(A);
  const size_t n = A.cols();
  if (inf_mask) {
    for (size_t j=0; j<n; ++j) {
      const double& vj = v(j);
      if (std::isfinite(vj))
        M.col(j) *= vj;
    }
  } else {
    for (size_t j=0; j<n; ++j)
      M.col(j) *= v(j);
  }
  return M;
}

/* ************************************************************************* */
Matrix LLt(const Matrix& A)
{
  Eigen::LLT<Matrix> llt(A);
  return llt.matrixL();
}

/* ************************************************************************* */
Matrix RtR(const Matrix &A)
{
  Eigen::LLT<Matrix> llt(A);
  return llt.matrixU();
}

/*
 * This is not ultra efficient, but not terrible, either.
 */
Matrix cholesky_inverse(const Matrix &A)
{
  Eigen::LLT<Matrix> llt(A);
  Matrix inv = Matrix::Identity(A.rows(),A.rows());
  llt.matrixU().solveInPlace<Eigen::OnTheRight>(inv);
  return inv*inv.transpose();
}

/* ************************************************************************* */
// Semantics:
// if B = inverse_square_root(A), then all of the following are true:
// inv(B) * inv(B)' == A
// inv(B' * B) == A
Matrix inverse_square_root(const Matrix& A) {
  Eigen::LLT<Matrix> llt(A);
  Matrix inv = Matrix::Identity(A.rows(),A.rows());
  llt.matrixU().solveInPlace<Eigen::OnTheRight>(inv);
  return inv.transpose();
}

/* ************************************************************************* */
void svd(const Matrix& A, Matrix& U, Vector& S, Matrix& V) {
  Eigen::JacobiSVD<Matrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  U = svd.matrixU();
  S = svd.singularValues();
  V = svd.matrixV();
}

/* ************************************************************************* */
boost::tuple<int, double, Vector> DLT(const Matrix& A, double rank_tol) {

  // Check size of A
  size_t n = A.rows(), p = A.cols(), m = min(n,p);

  // Do SVD on A
  Eigen::JacobiSVD<Matrix> svd(A, Eigen::ComputeFullV);
  Vector s = svd.singularValues();
  Matrix V = svd.matrixV();

  // Find rank
  size_t rank = 0;
  for (size_t j = 0; j < m; j++)
    if (s(j) > rank_tol) rank++;

  // Return rank, error, and corresponding column of V
  double error = m<p ? 0 : s(m-1);
  return boost::tuple<int, double, Vector>((int)rank, error, Vector(column(V, p-1)));
}

/* ************************************************************************* */
Matrix expm(const Matrix& A, size_t K) {
  Matrix E = Matrix::Identity(A.rows(),A.rows()), A_k = Matrix::Identity(A.rows(),A.rows());
  for(size_t k=1;k<=K;k++) {
    A_k = A_k*A/double(k);
    E = E + A_k;
  }
  return E;
}

/* ************************************************************************* */
std::string formatMatrixIndented(const std::string& label, const Matrix& matrix, bool makeVectorHorizontal)
{
  stringstream ss;
  const string firstline = label;
  ss << firstline;
  const string padding(firstline.size(), ' ');
  const bool transposeMatrix = makeVectorHorizontal && matrix.cols() == 1 && matrix.rows() > 1;
  const DenseIndex effectiveRows = transposeMatrix ? matrix.cols() : matrix.rows();

  if(matrix.rows() > 0 && matrix.cols() > 0)
  {
    stringstream matrixPrinted;
    if(transposeMatrix)
      matrixPrinted << matrix.transpose();
    else
      matrixPrinted << matrix;
    const std::string matrixStr = matrixPrinted.str();
    boost::tokenizer<boost::char_separator<char> > tok(matrixStr, boost::char_separator<char>("\n"));

    DenseIndex row = 0;
    for(const std::string& line: tok)
    {
      assert(row < effectiveRows);
      if(row > 0)
        ss << padding;
      ss << "[ " << line << " ]";
      if(row < effectiveRows - 1)
        ss << "\n";
      ++ row;
    }
  } else {
    ss << "Empty (" << matrix.rows() << "x" << matrix.cols() << ")";
  }
  return ss.str();
}

/* ************************************************************************* */
void inplace_QR(Matrix& A){
  size_t rows = A.rows();
  size_t cols = A.cols();
  size_t size = std::min(rows,cols);

  typedef Eigen::internal::plain_diag_type<Matrix>::type HCoeffsType;
  typedef Eigen::internal::plain_row_type<Matrix>::type RowVectorType;
  HCoeffsType hCoeffs(size);
  RowVectorType temp(cols);

#if !EIGEN_VERSION_AT_LEAST(3,2,5)
  Eigen::internal::householder_qr_inplace_blocked<Matrix, HCoeffsType>(A, hCoeffs, 48, temp.data());
#else
  Eigen::internal::householder_qr_inplace_blocked<Matrix, HCoeffsType>::run(A, hCoeffs, 48, temp.data());
#endif

  zeroBelowDiagonal(A);
}

} // namespace gtsam
