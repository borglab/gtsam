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

#include <gtsam/base/types.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/FastList.h>

#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/3rdparty/Eigen/Eigen/SVD>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include <cstdarg>
#include <cstring>
#include <iomanip>
#include <list>
#include <fstream>
#include <limits>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Matrix Matrix_( size_t m, size_t n, const double* const data) {
	MatrixRowMajor A(m,n);
	copy(data, data+m*n, A.data());
	return Matrix(A);
}

/* ************************************************************************* */
Matrix Matrix_( size_t m, size_t n, const Vector& v)
{
	Matrix A(m,n);
	// column-wise copy
	for( size_t j = 0, k=0  ; j < n ; j++)
		for( size_t i = 0; i < m ; i++,k++)
			A(i,j) = v(k);
	return A;
}

/* ************************************************************************* */
Matrix Matrix_(size_t m, size_t n, ...) {
	Matrix A(m,n);
	va_list ap;
	va_start(ap, n);
	for( size_t i = 0 ; i < m ; i++)
		for( size_t j = 0 ; j < n ; j++) {
			double value = va_arg(ap, double);
			A(i,j) = value;
		}
	va_end(ap);
	return A;
}

/* ************************************************************************* */
Matrix zeros( size_t m, size_t n ) {
	return Matrix::Zero(m,n);
}

/* ************************************************************************* */
Matrix eye( size_t m, size_t n) {
	return Matrix::Identity(m, n);
}

/* ************************************************************************* */ 
Matrix diag(const Vector& v) {
	return v.asDiagonal();
}

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
void multiplyAdd(double alpha, const Matrix& A, const Vector& x, Vector& e) {
	e += alpha * A * x;
}

/* ************************************************************************* */
void multiplyAdd(const Matrix& A, const Vector& x, Vector& e) {
	e += A * x;
}

/* ************************************************************************* */
Vector operator^(const Matrix& A, const Vector & v) {
	if (A.rows()!=v.size()) throw std::invalid_argument(
			boost::str(boost::format("Matrix operator^ : A.m(%d)!=v.size(%d)") % A.rows() % v.size()));
//	Vector vt = v.transpose();
//	Vector vtA = vt * A;
//	return vtA.transpose();
	return A.transpose() * v;
}

/* ************************************************************************* */
void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, Vector& x) {
	x += alpha * A.transpose() * e;
}

/* ************************************************************************* */
void transposeMultiplyAdd(const Matrix& A, const Vector& e, Vector& x) {
	x += A.transpose() * e;
}

/* ************************************************************************* */
void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, SubVector x) {
	x += alpha * A.transpose() * e;
}

/* ************************************************************************* */
Vector Vector_(const Matrix& A)
{
	size_t m = A.rows(), n = A.cols();
	Vector v(m*n);
	for( size_t j = 0, k=0  ; j < n ; j++)
		for( size_t i = 0; i < m ; i++,k++)
			v(k) = A(i,j);
	return v;
}

/* ************************************************************************* */
void print(const Matrix& A, const string &s, ostream& stream) {
	size_t m = A.rows(), n = A.cols();

	// print out all elements
	stream << s << "[\n";
	for( size_t i = 0 ; i < m ; i++) {
		for( size_t j = 0 ; j < n ; j++) {
			double aij = A(i,j);
			if(aij != 0.0)
				stream << setw(12) << setprecision(9) << aij << ",\t";
			else
				stream << "         0.0,\t";
		}
		stream << endl;
	}
	stream << "];" << endl;
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
  BOOST_FOREACH(const vector<double>& rowVec, coeffs) {
    destinationMatrix.row(row) = Eigen::Map<const Eigen::RowVectorXd>(&rowVec[0], width);
    ++ row;
  }

  return inputStream;
}

/* ************************************************************************* */
void insertSub(Matrix& fullMatrix, const Matrix& subMatrix, size_t i, size_t j) {
	fullMatrix.block(i, j, subMatrix.rows(), subMatrix.cols()) = subMatrix;
}

/* ************************************************************************* */
void insertColumn(Matrix& A, const Vector& col, size_t j) {
	A.col(j) = col;
}

/* ************************************************************************* */
void insertColumn(Matrix& A, const Vector& col, size_t i, size_t j) {
	A.col(j).segment(i, col.size()) = col;
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
void solve(Matrix& A, Matrix& B) {
	// Eigen version - untested
	B = A.fullPivLu().solve(B);
}

/* ************************************************************************* */
Matrix inverse(const Matrix& A) {
	return A.inverse();
}

/* ************************************************************************* */
/** Householder QR factorization, Golub & Van Loan p 224, explicit version    */
/* ************************************************************************* */
pair<Matrix,Matrix> qr(const Matrix& A) {
	const size_t m = A.rows(), n = A.cols(), kprime = min(m,n);

	Matrix Q=eye(m,m),R(A);
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
		Matrix Qj = eye(m) - beta * v * v.transpose();

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
	Vector weights = reciprocal(emul(sigmas,sigmas)); // calculate weights once

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
		Vector r(basis(n, j));
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
		tic(1, "householder_update"); // bottleneck for system
		// don't touch old columns
		Vector w = beta * A.block(j,j,m-j,n-j).transpose() * vjm;
		A.block(j,j,m-j,n-j) -= vjm * w.transpose();
		toc(1, "householder_update");

		// the Householder vector is copied in the zeroed out part
		if (copy_vectors) {
			tic(2, "householder_vector_copy");
			A.col(j).segment(j+1, m-(j+1)) = vjm.segment(1, m-(j+1));
			toc(2, "householder_vector_copy");
		}
	} // column j
}

/* ************************************************************************* */
void householder(Matrix& A, size_t k) {
	// version with zeros below diagonal
	tic(1, "householder_");
	householder_(A,k,false);
	toc(1, "householder_");
//	tic(2, "householder_zero_fill");
//	const size_t m = A.rows(), n = A.cols(), kprime = min(k,min(m,n));
//	for(size_t j=0; j < kprime; j++)
//		A.col(j).segment(j+1, m-(j+1)).setZero();
//	toc(2, "householder_zero_fill");
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
Matrix collect(const std::vector<const Matrix *>& matrices, size_t m, size_t n)
{
	// if we have known and constant dimensions, use them
	size_t dimA1 = m;
	size_t dimA2 = n*matrices.size();
	if (!m && !n) {
		BOOST_FOREACH(const Matrix* M, matrices) {
			dimA1 =  M->rows();  // TODO: should check if all the same !
			dimA2 += M->cols();
		}
	}

	// stl::copy version
	Matrix A(dimA1, dimA2);
	size_t hindex = 0;
	BOOST_FOREACH(const Matrix* M, matrices) {
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
	const size_t m = A.rows();
	if (inf_mask) {
		for (size_t i=0; i<m; ++i) {
			const double& vi = v(i);
			if (!isnan(vi) && !isinf(vi))
				A.row(i) *= vi;
		}
	} else {
		for (size_t i=0; i<m; ++i)
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
			if (!isnan(vj) && !isinf(vj))
				M.col(j) *= vj;
		}
	} else {
		for (size_t j=0; j<n; ++j)
			M.col(j) *= v(j);
	}
	return M;
}

/* ************************************************************************* */
Matrix3 skewSymmetric(double wx, double wy, double wz)
{
	return (Matrix3() <<
			0.0, -wz, +wy,
			+wz, 0.0, -wx,
			-wy, +wx, 0.0).finished();
}

/* ************************************************************************* */
/** Numerical Recipes in C wrappers                                          
 *  create Numerical Recipes in C structure
 * pointers are subtracted by one to provide base 1 access 
 */
/* ************************************************************************* */
// FIXME: assumes row major, rather than column major
//double** createNRC(Matrix& A) {
//	const size_t m=A.rows();
//	double** a = new double* [m];
//	for(size_t i = 0; i < m; i++)
//		a[i] = &A(i,0)-1;
//	return a;
//}

/* ******************************************
 * 
 * Modified from Justin's codebase
 *
 *  Idea came from other public domain code.  Takes a S.P.D. matrix
 *  and computes the LL^t decomposition.  returns L, which is lower
 *  triangular.  Note this is the opposite convention from Matlab,
 *  which calculates Q'Q where Q is upper triangular.
 *
 * ******************************************/
Matrix LLt(const Matrix& A)
{
	Matrix L = zeros(A.rows(), A.rows());
	Eigen::LLT<Matrix> llt;
	llt.compute(A);
	return llt.matrixL();
}

Matrix RtR(const Matrix &A)
{
	return LLt(A).transpose();
}

/*
 * This is not ultra efficient, but not terrible, either.
 */
Matrix cholesky_inverse(const Matrix &A)
{
	// FIXME: replace with real algorithm
	return A.inverse();

//	Matrix L = LLt(A);
//	Matrix inv(eye(A.rows()));
//	inplace_solve (L, inv, BNU::lower_tag ());
//	return BNU::prod(trans(inv), inv);
}

#if 0
/* ************************************************************************* */
// TODO, would be faster with Cholesky
Matrix inverse_square_root(const Matrix& A) {
	size_t m = A.cols(), n = A.rows();
	if (m!=n)
		throw invalid_argument("inverse_square_root: A must be square");

	// Perform SVD, TODO: symmetric SVD?
	Matrix U,V;
	Vector S;
	svd(A,U,S,V);

	// invert and sqrt diagonal of S
	// We also arbitrarily choose sign to make result have positive signs
	for(size_t i = 0; i<m; i++) S(i) = - pow(S(i),-0.5);
	return vector_scale(S, V); // V*S;
}
#endif

/* ************************************************************************* */
// New, improved, with 100% more Cholesky goodness!
//
// Semantics: 
// if B = inverse_square_root(A), then all of the following are true:
// inv(B) * inv(B)' == A
// inv(B' * B) == A
Matrix inverse_square_root(const Matrix& A) {
	Matrix R = RtR(A);
	Matrix inv = eye(A.rows());
	R.triangularView<Eigen::Upper>().solveInPlace<Eigen::OnTheRight>(inv);
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
	int n = A.rows(), p = A.cols(), m = min(n,p);

	// Do SVD on A
  Eigen::JacobiSVD<Matrix> svd(A, Eigen::ComputeFullV);
  Vector s = svd.singularValues();
  Matrix V = svd.matrixV();

	// Find rank
	int rank = 0;
	for (int j = 0; j < m; j++)
		if (s(j) > rank_tol) rank++;

	// Return rank, error, and corresponding column of V
	double error = m<p ? 0 : s(m-1);
	return boost::tuple<int, double, Vector>(rank, error, Vector(column(V, p-1)));
}

/* ************************************************************************* */
Matrix expm(const Matrix& A, size_t K) {
	Matrix E = eye(A.rows()), A_k = eye(A.rows());
	for(size_t k=1;k<=K;k++) {
		A_k = A_k*A/k;
		E = E + A_k;
	}
	return E;
}

/* ************************************************************************* */
Matrix Cayley(const Matrix& A) {
	int n = A.cols();
	assert(A.rows() == n);

	// original
//	const Matrix I = eye(n);
//	return (I-A)*inverse(I+A);

	// inlined to let Eigen do more optimization
	return (Matrix::Identity(n, n) - A)*(Matrix::Identity(n, n) + A).inverse();
}
/* ************************************************************************* */

} // namespace gtsam
