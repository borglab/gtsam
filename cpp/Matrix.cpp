/**
 * @file   Matrix.cpp
 * @brief  matrix class
 * @author Christian Potthast
 */

#include <stdarg.h>
#include <string.h>
#include <iomanip>
#include <list>
#include <fstream>

// TESTING
#ifdef CBLAS
#include <cblas.h>
#endif

#ifdef GSL
#include <gsl/gsl_blas.h> // needed for gsl blas
#include <gsl/gsl_linalg.h>
#endif

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/foreach.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/tuple/tuple.hpp>

#include "Matrix.h"
#include "Vector.h"
#include "svdcmp.h"

// use for switching quickly between GSL and nonGSL versions without reconfigure
//#define REVERTGSL


using namespace std;
namespace ublas = boost::numeric::ublas;

namespace gtsam {

/* ************************************************************************* */
Matrix Matrix_( size_t m, size_t n, const double* const data) {
  Matrix A(m,n);
  copy(data, data+m*n, A.data().begin());
  return A;
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
/** create a matrix with value zero                                          */
/* ************************************************************************* */
Matrix zeros( size_t m, size_t n )
{
  Matrix A(m,n, 0.0);
  return A;
}

/** 
 * Identity matrix
 */
Matrix eye( size_t m, size_t n){
  Matrix A = zeros(m,n);
  for(size_t i = 0; i<min(m,n); i++) A(i,i)=1.0;
  return A;
}

/* ************************************************************************* */ 
/** Diagonal matrix values                                                   */
/* ************************************************************************* */
Matrix diag(const Vector& v) {
  size_t m = v.size();
  Matrix A = zeros(m,m);
  for(size_t i = 0; i<m; i++) A(i,i)=v(i);
  return A;
}

/* ************************************************************************* */
/** Check if two matrices are the same                                       */
/* ************************************************************************* */
bool equal_with_abs_tol(const Matrix& A, const Matrix& B, double tol) {

  size_t n1 = A.size2(), m1 = A.size1();
  size_t n2 = B.size2(), m2 = B.size1();

  if(m1!=m2 || n1!=n2) return false;

  for(size_t i=0; i<m1; i++)
	  for(size_t j=0; j<n1; j++) {
		  if(isnan(A(i,j)) xor isnan(B(i,j)))
			  return false;
		  if(fabs(A(i,j) - B(i,j)) > tol)
			  return false;
	  }

  return true;
}

/* ************************************************************************* */
bool assert_equal(const Matrix& expected, const Matrix& actual, double tol) {

  if (equal_with_abs_tol(expected,actual,tol)) return true;

  size_t n1 = expected.size2(), m1 = expected.size1();
  size_t n2 = actual.size2(), m2 = actual.size1();

  cout << "not equal:" << endl;
  print(expected,"expected = ");
  print(actual,"actual = ");
  if(m1!=m2 || n1!=n2)
    cout << m1 << "," << n1 << " != " << m2 << "," << n2 << endl;
  else
    print(actual-expected, "actual - expected = ");
  return false;
}

/* ************************************************************************* */
void multiplyAdd(double alpha, const Matrix& A, const Vector& x, Vector& e) {
#if defined CBLAS

	// get sizes
	const size_t m = A.size1(), n = A.size2();

	// get pointers
	const double * Aptr = A.data().begin();
	const double * Xptr = x.data().begin();
	double * Eptr = e.data().begin();

	// fill in parameters
	const double beta = 1.0;
	const int incx = 1, incy = 1, ida = n;

	// execute blas call
	cblas_dgemv(CblasRowMajor, CblasNoTrans, m, n, alpha, Aptr, ida, Xptr, incx, beta, Eptr, incy);

#elif defined GSL
	gsl_vector_const_view xg = gsl_vector_const_view_array(x.data().begin(), x.size());
	gsl_vector_view eg = gsl_vector_view_array(e.data().begin(), e.size());
	gsl_matrix_const_view Ag = gsl_matrix_const_view_array(A.data().begin(), A.size1(), A.size2());
	gsl_blas_dgemv (CblasNoTrans, alpha, &(Ag.matrix), &(xg.vector), 1.0, &(eg.vector));
#else
	// ublas e += prod(A,x) is terribly slow
  size_t m = A.size1(), n = A.size2();
	double * ei = e.data().begin();
	const double * aij = A.data().begin();
	for (int i = 0; i < m; i++, ei++) {
		const double * xj = x.data().begin();
		for (int j = 0; j < n; j++, aij++, xj++)
			(*ei) += alpha * (*aij) * (*xj);
	}
#endif
}

/* ************************************************************************* */
void multiplyAdd(const Matrix& A, const Vector& x, Vector& e) {
	// ublas e += prod(A,x) is terribly slow
#ifdef CBLAS
	multiplyAdd(1.0, A, x, e);
#else
  size_t m = A.size1(), n = A.size2();
	double * ei = e.data().begin();
	const double * aij = A.data().begin();
	for (int i = 0; i < m; i++, ei++) {
		const double * xj = x.data().begin();
		for (int j = 0; j < n; j++, aij++, xj++)
			(*ei) += (*aij) * (*xj);
	}
#endif
	}

/* ************************************************************************* */
Vector operator^(const Matrix& A, const Vector & v) {
  if (A.size1()!=v.size()) throw std::invalid_argument(
  		boost::str(boost::format("Matrix operator^ : A.m(%d)!=v.size(%d)") % A.size1() % v.size()));
  Vector vt = trans(v);
  Vector vtA = prod(vt,A);
  return trans(vtA);
}

/* ************************************************************************* */
void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, Vector& x) {
#if defined CBLAS

	// get sizes
	const size_t m = A.size1(), n = A.size2();

	// get pointers
	const double * Aptr = A.data().begin();
	const double * Xptr = e.data().begin();
	double * Eptr = x.data().begin();

	// fill in parameters
	const double beta = 1.0;
	const int incx = 1, incy = 1, ida = n;

	// execute blas call
	cblas_dgemv(CblasRowMajor, CblasTrans, m, n, alpha, Aptr, ida, Xptr, incx, beta, Eptr, incy);

#elif defined GSL
	gsl_vector_const_view eg = gsl_vector_const_view_array(e.data().begin(), e.size());
	gsl_vector_view xg = gsl_vector_view_array(x.data().begin(), x.size());
	gsl_matrix_const_view Ag = gsl_matrix_const_view_array(A.data().begin(), A.size1(), A.size2());
	gsl_blas_dgemv (CblasTrans, alpha, &(Ag.matrix), &(eg.vector), 1.0, &(xg.vector));
#else
	// ublas x += prod(trans(A),e) is terribly slow
	// TODO: use BLAS
  size_t m = A.size1(), n = A.size2();
	double * xj = x.data().begin();
	for (int j = 0; j < n; j++,xj++) {
		const double * ei = e.data().begin();
		const double * aij = A.data().begin() + j;
		for (int i = 0; i < m; i++, aij+=n, ei++)
			(*xj) += alpha * (*aij) * (*ei);
	}
#endif
}

/* ************************************************************************* */
void transposeMultiplyAdd(const Matrix& A, const Vector& e, Vector& x) {
	// ublas x += prod(trans(A),e) is terribly slow
#ifdef CBLAS
	transposeMultiplyAdd(1.0, A, e, x);
#else
  size_t m = A.size1(), n = A.size2();
	double * xj = x.data().begin();
	for (int j = 0; j < n; j++,xj++) {
		const double * ei = e.data().begin();
		const double * aij = A.data().begin() + j;
		for (int i = 0; i < m; i++, aij+=n, ei++)
			(*xj) += (*aij) * (*ei);
	}
#endif
}

/* ************************************************************************* */
void transposeMultiplyAdd(double alpha, const Matrix& A, const Vector& e, SubVector x) {
	// ublas x += prod(trans(A),e) is terribly slow
	// TODO: use BLAS
  size_t m = A.size1(), n = A.size2();
	for (int j = 0; j < n; j++) {
		const double * ei = e.data().begin();
		const double * aij = A.data().begin() + j;
		for (int i = 0; i < m; i++, aij+=n, ei++)
			x(j) += alpha * (*aij) * (*ei);
	}
}

/* ************************************************************************* */
Vector Vector_(const Matrix& A)
{
  size_t m = A.size1(), n = A.size2();
  Vector v(m*n);
  for( size_t j = 0, k=0  ; j < n ; j++)
    for( size_t i = 0; i < m ; i++,k++)
      v(k) = A(i,j);
  return v;
}

/* ************************************************************************* */
Vector column_(const Matrix& A, size_t j) {
//	if (j>=A.size2())
//		throw invalid_argument("Column index out of bounds!");

	return column(A,j); // real boost version

	// TODO: improve this
//	size_t m = A.size1();
//	Vector a(m);
//	for (size_t i=0; i<m; ++i)
//		a(i) = A(i,j);
//	return a;
}

/* ************************************************************************* */
Vector row_(const Matrix& A, size_t i) {
	if (i>=A.size1())
		throw invalid_argument("Row index out of bounds!");

	const double * Aptr = A.data().begin() + A.size2() * i;
	return Vector_(A.size2(), Aptr);
}

/* ************************************************************************* */
void print(const Matrix& A, const string &s, ostream& stream) {
  size_t m = A.size1(), n = A.size2();

  // print out all elements
  stream << s << "[\n";
  for( size_t i = 0 ; i < m ; i++) {
    for( size_t j = 0 ; j < n ; j++) {
      double aij = A(i,j);
      stream << setw(9) << (fabs(aij)<1e-12 ? 0 : aij) << "\t";
    }
    stream << endl;
  }
  stream << "]" << endl;
}

/* ************************************************************************* */
void save(const Matrix& A, const string &s, const string& filename) {
	fstream stream(filename.c_str(), fstream::out);
	print(A, s + "=", stream);
	stream.close();
}

/* ************************************************************************* */
Matrix sub(const Matrix& A, size_t i1, size_t i2, size_t j1, size_t j2) {
  // using ublas is slower:
  // Matrix B = Matrix(ublas::project(A,ublas::range(i1,i2+1),ublas::range(j1,j2+1)));
  size_t m=i2-i1, n=j2-j1;
  Matrix B(m,n);
  for (size_t i=i1,k=0;i<i2;i++,k++)
    memcpy(&B(k,0),&A(i,j1),n*sizeof(double));
  return B;
}

/* ************************************************************************* */
void insertSub(Matrix& big, const Matrix& small, size_t i, size_t j) {
	// direct pointer method
	size_t ib = big.size1(), jb = big.size2(),
		   is = small.size1(), js = small.size2();

	// pointer to start of window in big
	double * bigptr = big.data().begin() + i*jb + j;
	const double * smallptr = small.data().begin();
	for (size_t row=0; row<is; ++row)
		memcpy(bigptr+row*jb, smallptr+row*js, js*sizeof(double));
}

/* ************************************************************************* */
void insertColumn(Matrix& A, const Vector& col, size_t j) {
	ublas::matrix_column<Matrix> colproxy(A, j);
	colproxy = col;
}

/* ************************************************************************* */
void insertColumn(Matrix& A, const Vector& col, size_t i, size_t j) {
	ublas::matrix_column<Matrix> colproxy(A, j);
	ublas::vector_range<ublas::matrix_column<Matrix> > colsubproxy(colproxy,
			ublas::range (i, i+col.size()));
	colsubproxy = col;
}

/* ************************************************************************* */
void solve(Matrix& A, Matrix& B)
{
	typedef ublas::permutation_matrix<std::size_t> pmatrix;
	// create a working copy of the input
	Matrix A_(A);
	// create a permutation matrix for the LU-factorization
	pmatrix pm(A_.size1());

	// perform LU-factorization
	int res = lu_factorize(A_,pm);
	if( res != 0 ) throw runtime_error ("Matrix::solve: lu_factorize failed!");

	// backsubstitute to get the inverse
	lu_substitute(A_, pm, B);
}

/* ************************************************************************* */
Matrix inverse(const Matrix& originalA)
{
  Matrix A(originalA);
  Matrix B = eye(A.size2());
  solve(A,B);
  return B;
}

/* ************************************************************************* */
/** Householder QR factorization, Golub & Van Loan p 224, explicit version    */
/* ************************************************************************* */
pair<Matrix,Matrix> qr(const Matrix& A) {

  const size_t m = A.size1(), n = A.size2(), kprime = min(m,n);
  
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
    Matrix Qj = eye(m) - beta * Matrix(outer_prod(v,v)); //BAD: Fix this

    R = Qj * R; // update R
    Q = Q * Qj; // update Q

  } // column j

  return make_pair(Q,R);
}

/* ************************************************************************* */
/** Imperative version of Householder rank 1 update
 * i.e. do outer product update A = (I-beta vv')*A = A - v*(beta*A'*v)' = A - v*w'
 * but only in relevant part, from row j onwards
 * If called from householder_ does actually more work as first j columns 
 * will not be touched. However, is called from GaussianFactor.eliminate
 * on a number of different matrices for which all columns change.
 */
/* ************************************************************************* */
inline void householder_update_manual(Matrix &A, int j, double beta, const Vector& vjm) {
	const size_t m = A.size1(), n = A.size2();
	// w = beta*transpose(A(j:m,:))*v(j:m)
	Vector w(n);
	for( size_t c = 0; c < n; c++) {
		w(c) = 0.0;
		// dangerous as relies on row-major scheme
		const double *a = &A(j,c), * const v = &vjm(0);
		for( size_t r=j, s=0 ; r < m ; r++, s++, a+=n )
			// w(c) += A(r,c) * vjm(r-j)
			w(c) += (*a) * v[s];
		w(c) *= beta;
	}

	// rank 1 update A(j:m,:) -= v(j:m)*w'
	for( size_t c = 0 ; c < n; c++) {
		double wc = w(c);
		double *a = &A(j,c); const double * const v =&vjm(0);
		for( size_t r=j, s=0 ; r < m ; r++, s++, a+=n )
			// A(r,c) -= vjm(r-j) * wjn(c-j);
			(*a) -= v[s] * wc;
	}
}

void householder_update(Matrix &A, int j, double beta, const Vector& vjm) {
	// TODO: SWAP IN ATLAS VERSION OF THE SYSTEM
//	// straight atlas version
//	const size_t m = A.size1(), n = A.size2(), mj = m-j;
//
//	// find pointers to the data
//	const double * vptr = vjm.data().begin(); // mj long
//	double * Aptr = A.data().begin() + n*j; // mj x n - note that this starts at row j
//
//	// first step: get w = beta*trans(A(j:m,:))*vjm
//	Vector w(n);
//	double * wptr = w.data().begin();
//
//	// execute w generation
//	cblas_dgemv(CblasRowMajor, CblasTrans, mj, n, beta, Aptr, n, vptr, 1, 0.0, wptr, 1);
//
//	// second step: rank 1 update A(j:m,:) = v(j:m)*w' + A(j:m,:)
//	cblas_dger(CblasRowMajor, mj, n, 1.0, vptr, 1, wptr, 1, Aptr, n);


#ifdef GSL
#ifndef REVERTGSL
	const size_t m = A.size1(), n = A.size2();
	// use GSL version
	gsl_vector_const_view v = gsl_vector_const_view_array(vjm.data().begin(), m-j);
	gsl_matrix_view Ag = gsl_matrix_view_array(A.data().begin(), m, n);
	gsl_matrix_view Ag_view = gsl_matrix_submatrix (&(Ag.matrix), j, 0, m-j, n);
	gsl_linalg_householder_hm (beta, &(v.vector), &(Ag_view.matrix));
#else
	householder_update_manual(A,j,beta,vjm);
#endif

#else
	householder_update_manual(A,j,beta,vjm);
#endif
}

/* ************************************************************************* */
// update A, b
// A' \define A_{S}-ar and b'\define b-ad
// __attribute__ ((noinline))	// uncomment to prevent inlining when profiling
inline void updateAb_manual(Matrix& A, Vector& b, int j, const Vector& a,
		const Vector& r, double d) {
	const size_t m = A.size1(), n = A.size2();
	for (size_t i = 0; i < m; i++) { // update all rows
		double ai = a(i);
		b(i) -= ai * d;
		double *Aij = A.data().begin() + i * n + j + 1;
		const double *rptr = r.data().begin() + j + 1;
		// A(i,j+1:end) -= ai*r(j+1:end)
		for (size_t j2 = j + 1; j2 < n; j2++, Aij++, rptr++)
			*Aij -= ai * (*rptr);
	}
}

static void updateAb(Matrix& A, Vector& b, int j, const Vector& a,
		const Vector& r, double d) {
#ifdef GSL
#ifndef REVERTGSL
	const size_t m = A.size1(), n = A.size2();
	// update A
	// A(0:m,j+1:end) = A(0:m,j+1:end) - a(0:m)*r(j+1:end)'
	// get a view for A
	gsl_matrix_view Ag = gsl_matrix_view_array(A.data().begin(), m, n);
	gsl_matrix_view Ag_view = gsl_matrix_submatrix (&(Ag.matrix), 0, j+1, m, n-j-1);
	// get a view for r
	gsl_vector_const_view rg = gsl_vector_const_view_array(r.data().begin()+j+1, n-j-1);
	// get a view for a
	gsl_vector_const_view ag = gsl_vector_const_view_array(a.data().begin(), m);

	// rank one update
	gsl_blas_dger (-1.0, &(ag.vector), &(rg.vector), &(Ag_view.matrix));

	// update b
	double * bptr = b.data().begin();
	const double * aptr = a.data().begin();
	for (size_t i = 0; i < m; i++) {
		*(bptr+i) -= d* *(aptr+i);
	}

#else
	updateAb_manual(A,b,j,a,r,d);
#endif

#else
	updateAb_manual(A,b,j,a,r,d);
#endif
}

/* ************************************************************************* */
list<boost::tuple<Vector, double, double> >
weighted_eliminate(Matrix& A, Vector& b, const Vector& sigmas) {
	size_t m = A.size1(), n = A.size2(); // get size(A)
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
		Vector a(column_(A, j)); // ublas::matrix_column is slower !
		//print(a,"a");

		// Calculate weighted pseudo-inverse and corresponding precision
		double precision = weightedPseudoinverse(a, weights, pseudo);
//		cout << precision << endl;
//		print(pseudo,"pseudo");

		// if precision is zero, no information on this column
		if (precision < 1e-8) continue;

		// create solution and copy into r
		Vector r(basis(n, j));
		for (size_t j2=j+1; j2<n; ++j2)
			r(j2) = inner_prod(pseudo, ublas::matrix_column<Matrix>(A, j2)); // TODO: don't use ublas

		// create the rhs
		double d = inner_prod(pseudo, b);

		// construct solution (r, d, sigma)
		// TODO: avoid sqrt, store precision or at least variance
		results.push_back(boost::make_tuple(r, d, 1./sqrt(precision)));

		// exit after rank exhausted
		if (results.size()>=maxRank) break;

		// update A, b, expensive, using outer product
		// A' \define A_{S}-a*r and b'\define b-d*a
		updateAb(A, b, j, a, r, d);
	}

	return results;
}

/* ************************************************************************* */
/** Imperative version of Householder QR factorization, Golub & Van Loan p 224
 * version with Householder vectors below diagonal, as in GVL
 */
/* ************************************************************************* */
inline void householder_manual(Matrix &A, size_t k) {
	const size_t m = A.size1(), n = A.size2(), kprime = min(k,min(m,n));
	// loop over the kprime first columns
	for(size_t j=0; j < kprime; j++){
		// below, the indices r,c always refer to original A

		// copy column from matrix to xjm, i.e. x(j:m) = A(j:m,j)
		Vector xjm(m-j);
		for(size_t r = j ; r < m; r++)
			xjm(r-j) = A(r,j);

		// calculate the Householder vector
		double beta; Vector vjm;
		boost::tie(beta,vjm) = house(xjm);

		// do outer product update A = (I-beta vv')*A = A - v*(beta*A'*v)' = A - v*w'
		householder_update(A, j, beta, vjm);

		// the Householder vector is copied in the zeroed out part
		for( size_t r = j+1 ; r < m ; r++ )
			A(r,j) = vjm(r-j);

	} // column j
}

void householder_(Matrix &A, size_t k) 
{
#ifdef GSL
#ifndef REVERTGSL
	const size_t m = A.size1(), n = A.size2(), kprime = min(k,min(m,n));
	// loop over the kprime first columns
	for(size_t j=0; j < kprime; j++){
		// below, the indices r,c always refer to original A

		// copy column from matrix to xjm, i.e. x(j:m) = A(j:m,j)
		Vector xjm(m-j);
		for(size_t r = j ; r < m; r++)
			xjm(r-j) = A(r,j);

		// calculate the Householder vector
		// COPIED IN: boost::tie(beta,vjm) = house(xjm);
		const double x0 = xjm(0);
		const double x02 = x0*x0;

		const double sigma = inner_prod(trans(xjm),xjm) - x02;
		double beta = 0.0; Vector vjm(xjm);  vjm(0) = 1.0;

		if( sigma == 0.0 )
			beta = 0.0;
		else {
			double mu = sqrt(x02 + sigma);
			if( x0 <= 0.0 )
				vjm(0) = x0 - mu;
			else
				vjm(0) = -sigma / (x0 + mu);

			const double v02 = vjm(0)*vjm(0);
			beta = 2.0 * v02 / (sigma + v02);
			vjm = vjm / vjm(0);
		}

		// do outer product update A = (I-beta vv')*A = A - v*(beta*A'*v)' = A - v*w'
		//householder_update(A, j, beta, vjm);

		// inlined use GSL version
		gsl_vector_const_view v = gsl_vector_const_view_array(vjm.data().begin(), m-j);
		gsl_matrix_view Ag = gsl_matrix_view_array(A.data().begin(), m, n);
		gsl_matrix_view Ag_view = gsl_matrix_submatrix (&(Ag.matrix), j, 0, m-j, n);
		gsl_linalg_householder_hm (beta, &(v.vector), &(Ag_view.matrix));

		// the Householder vector is copied in the zeroed out part
		for( size_t r = j+1 ; r < m ; r++ )
			A(r,j) = vjm(r-j);

	} // column j

#else
	householder_manual(A, k);
#endif

#else
	householder_manual(A, k);
#endif
}

/* ************************************************************************* */
/** version with zeros below diagonal                                        */
/* ************************************************************************* */
void householder(Matrix &A, size_t k) {
  householder_(A,k);
  const size_t m = A.size1(), n = A.size2(), kprime = min(k,min(m,n));
  for(size_t j=0; j < kprime; j++)
    for( size_t i = j+1 ; i < m ; i++ )
      A(i,j) = 0.0;
}

/* ************************************************************************* */
Vector backSubstituteUpper(const Matrix& U, const Vector& b, bool unit) {
	size_t m = U.size1(), n = U.size2();
#ifndef NDEBUG
	if (m!=n)
		throw invalid_argument("backSubstituteUpper: U must be square");
#endif

	Vector result(n);
	for (size_t i = n; i > 0; i--) {
		double zi = b(i-1);
		for (size_t j = i+1; j <= n; j++)
			zi -= U(i-1,j-1) * result(j-1);
		if (!unit) zi /= U(i-1,i-1);
		result(i-1) = zi;
	}

	return result;
}

/* ************************************************************************* */
Vector backSubstituteUpper(const Vector& b, const Matrix& U, bool unit) {
	size_t m = U.size1(), n = U.size2();
#ifndef NDEBUG
	if (m!=n)
		throw invalid_argument("backSubstituteUpper: U must be square");
#endif

	Vector result(n);
	for (size_t i = 1; i <= n; i++) {
		double zi = b(i-1);
		for (size_t j = 1; j < i; j++)
			zi -= U(j-1,i-1) * result(j-1);
		if (!unit) zi /= U(i-1,i-1);
		result(i-1) = zi;
	}

	return result;
}

/* ************************************************************************* */
Vector backSubstituteLower(const Matrix& L, const Vector& b, bool unit) {
	size_t m = L.size1(), n = L.size2();
#ifndef NDEBUG
	if (m!=n)
		throw invalid_argument("backSubstituteLower: L must be square");
#endif

	Vector result(n);
	for (size_t i = 1; i <= n; i++) {
		double zi = b(i-1);
		for (size_t j = 1; j < i; j++)
			zi -= L(i-1,j-1) * result(j-1);
		if (!unit) zi /= L(i-1,i-1);
		result(i-1) = zi;
	}

	return result;
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
    dimA1 += M->size1();
    dimA2 =  M->size2();  // TODO: should check if all the same !
  }
  va_end(ap);
  va_start(ap, nrMatrices);
  Matrix A(dimA1, dimA2);
  size_t vindex = 0;
  for( size_t i = 0 ; i < nrMatrices ; i++) {
    Matrix *M = va_arg(ap, Matrix *);
    for(size_t d1 = 0; d1 < M->size1(); d1++)
      for(size_t d2 = 0; d2 < M->size2(); d2++)
	A(vindex+d1, d2) = (*M)(d1, d2);
    vindex += M->size1();
  }  

  return A;
}

/* ************************************************************************* */
Matrix collect(const std::vector<const Matrix *>& matrices, size_t m, size_t n)
{
	// if we have known and constant dimensions, use them
	size_t dimA1 = m;
	size_t dimA2 = n*matrices.size();
	if (!m && !n)
		BOOST_FOREACH(const Matrix* M, matrices) {
		dimA1 =  M->size1();  // TODO: should check if all the same !
		dimA2 += M->size2();
	}

	// memcpy version
	Matrix A(dimA1, dimA2);
	double * Aptr = A.data().begin();
	size_t hindex = 0;
	BOOST_FOREACH(const Matrix* M, matrices) {
		size_t row_len = M->size2();

		// find the size of the row to copy
		size_t row_size = sizeof(double) * row_len;

		// loop over rows
		for(size_t d1 = 0; d1 < M->size1(); ++d1) { // rows
			// get a pointer to the start of the row in each matrix
			double * Arow = Aptr + d1*dimA2 + hindex;
			double * Mrow = const_cast<double*> (M->data().begin() + d1*row_len);

			// do direct memory copy to move the row over
			memcpy(Arow, Mrow, row_size);
		}
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
void vector_scale_inplace(const Vector& v, Matrix& A) {
	size_t m = A.size1(); size_t n = A.size2();
	for (size_t i=0; i<m; ++i) { // loop over rows
		double vi = v(i);
		double *Aij = A.data().begin() + i*n;
		for (size_t j=0; j<n; ++j, ++Aij) (*Aij) *= vi;
	}
}

/* ************************************************************************* */
// row scaling
Matrix vector_scale(const Vector& v, const Matrix& A) {
	Matrix M(A);
	vector_scale_inplace(v, M);
	return M;
}

/* ************************************************************************* */
// column scaling
Matrix vector_scale(const Matrix& A, const Vector& v) {
	Matrix M(A);
	size_t m = A.size1(); size_t n = A.size2();
	const double * vptr = v.data().begin();
	for (size_t i=0; i<m; ++i) { // loop over rows
		for (size_t j=0; j<n; ++j) { // loop over columns
			double * Mptr = M.data().begin() + i*n + j;
			(*Mptr) = (*Mptr) * *(vptr+j);
		}
	}
	return M;
}

/* ************************************************************************* */
Matrix skewSymmetric(double wx, double wy, double wz)
{
  return Matrix_(3,3,
		  0.0, -wz, +wy,
		  +wz, 0.0, -wx,
		  -wy, +wx, 0.0);
}

/* ************************************************************************* */
/** Numerical Recipes in C wrappers                                          
 *  create Numerical Recipes in C structure
 * pointers are subtracted by one to provide base 1 access 
 */
/* ************************************************************************* */
double** createNRC(Matrix& A) {
  const size_t m=A.size1();
  double** a = new double* [m];
  for(size_t i = 0; i < m; i++) 
    a[i] = &A(i,0)-1;
  return a;
}



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

namespace BNU = boost::numeric::ublas;


Matrix LLt(const Matrix& A)
{
	assert(A.size1() == A.size2());
        Matrix L = zeros(A.size1(), A.size1());

        for (size_t i = 0 ; i < A.size1(); i++) {
                double p = A(i,i) - BNU::inner_prod( BNU::project( BNU::row(L, i), BNU::range(0, i) ),
						     BNU::project( BNU::row(L, i), BNU::range(0, i) ) );
                assert(p > 0); // Rank failure
                double l_i_i = sqrt(p);
                L(i,i) = l_i_i;
                
		BNU::matrix_column<Matrix> l_i(L, i);
                project( l_i, BNU::range(i+1, A.size1()) )
                        = ( BNU::project( BNU::column(A, i), BNU::range(i+1, A.size1()) )
                            - BNU::prod( BNU::project(L, BNU::range(i+1, A.size1()), BNU::range(0, i)), 
					 BNU::project(BNU::row(L, i), BNU::range(0, i) ) ) ) / l_i_i;
        }
        return L;
}

Matrix RtR(const Matrix &A)
{
	return trans(LLt(A));
}

/*
 * This is not ultra efficient, but not terrible, either.
 */
Matrix cholesky_inverse(const Matrix &A)
{
        Matrix L = LLt(A);
        Matrix inv(boost::numeric::ublas::identity_matrix<double>(A.size1()));
        inplace_solve (L, inv, BNU::lower_tag ());
        return BNU::prod(trans(inv), inv);
}


/* ************************************************************************* */
/** SVD                                                                      */
/* ************************************************************************* */

// version with in place modification of A
void svd(Matrix& A, Vector& s, Matrix& V, bool sort) {

  const size_t m=A.size1(), n=A.size2();
  if( m < n )
	 throw invalid_argument("in-place svd calls NRC which needs matrix A with m>n");

  double * q = new double[n]; // singular values

  // create NRC matrices, u is in place
  V = Matrix(n,n);
  double **u = createNRC(A), **v = createNRC(V);

  // perform SVD
  // need to pass pointer - 1 in NRC routines so u[1][1] is first element !
  svdcmp(u-1,m,n,q-1,v-1, sort);
	
  // copy singular values back
  s.resize(n);
  copy(q,q+n,s.begin());

  delete[] v;
  delete[] q; //switched to array delete
  delete[] u;
}

/* ************************************************************************* */
void svd(const Matrix& A, Matrix& U, Vector& s, Matrix& V, bool sort) {
  const size_t m=A.size1(), n=A.size2();
  if( m < n ) {
	  V = trans(A);
	  svd(V,s,U,sort); // A'=V*diag(s)*U'
  }
  else{
	  U = A;      // copy
	  svd(U,s,V,sort); // call in-place version
  }
}

#if 0
/* ************************************************************************* */
// TODO, would be faster with Cholesky
Matrix inverse_square_root(const Matrix& A) {
  size_t m = A.size2(), n = A.size1();
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
        Matrix inv(boost::numeric::ublas::identity_matrix<double>(A.size1()));
        inplace_solve (R, inv, BNU::upper_tag ());
	return inv;
}



/* ************************************************************************* */
Matrix square_root_positive(const Matrix& A) {
  size_t m = A.size2(), n = A.size1();
  if (m!=n)
    throw invalid_argument("inverse_square_root: A must be square");

  // Perform SVD, TODO: symmetric SVD?
  Matrix U,V;
  Vector S;
  svd(A,U,S,V,false);

  // invert and sqrt diagonal of S
  // We also arbitrarily choose sign to make result have positive signs
  for(size_t i = 0; i<m; i++) S(i) = - pow(S(i),0.5);
  return vector_scale(S, V); // V*S;
}

/* ************************************************************************* */

} // namespace gtsam
