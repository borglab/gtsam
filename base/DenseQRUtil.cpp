/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DenseQRUtil.cpp
 *
 *   Created on: Jul 1, 2010
 *       Author: nikai
 *  Description: the utility functions for DenseQR
 */

#include <gtsam/base/timing.h>
#include <gtsam/base/DenseQRUtil.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>

using namespace std;
namespace ublas = boost::numeric::ublas;

#ifdef GT_USE_LAPACK
namespace gtsam {

	/* ************************************************************************* */
	long* MakeStairs(Matrix& A) {

		const long m = A.size1();
		const long n = A.size2();
		long* Stair = new long[n];

		// record the starting pointer of each row
		double* a[m];
		list<int> remained;
		a[0] = A.data().begin();
		for(int i=0; i<m-1; i++) {
			a[i+1] = a[i] + n;
			remained.push_back(i);
		}
		remained.push_back(m-1);

		// reorder the rows
		int j;
		vector<int> sorted;
		list<int>::iterator itRemained;
		for(j = 0; j < n; ) {
			// remove the non-zero rows in the current column
			for(itRemained = remained.begin(); itRemained!=remained.end(); ) {
				if (*(a[*itRemained]) != 0) {
					sorted.push_back(*itRemained);
					itRemained = remained.erase(itRemained);
				} else {
					a[*itRemained]++;
					itRemained ++;
				}
			}

			// record the stair number
			Stair[j++] = m - remained.size();

			if(remained.empty()) break;
		}

		// all the remained columns have maximum stair
		for (; j<n; j++)
			Stair[j] = m;

		// copy the new row
		Matrix A_new = zeros(m,n);
		int offset[m];
		offset[0] = 0;
		for(int i=1; i<m; i++)
			offset[i] = offset[i-1] +n;
		vector<int>::const_iterator itSorted;
		double* ptr1 = A.data().begin();
		double* ptr2 = A_new.data().begin();
		int row = 0, sizeOfRow = n * sizeof(double);
		for(itSorted=sorted.begin(); itSorted!=sorted.end(); itSorted++, row++)
			memcpy(ptr2+offset[row], ptr1+offset[*itSorted], sizeOfRow);

		A = A_new;

		return Stair;
	}

	void printColumnMajor(const double* a, const long m, const long n) {
		for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
				cout << a[j*m+i] << "\t";
			cout << endl;
		}
	}

	/* ************************************************************************* */
	void householder_denseqr(Matrix &A, long* Stair) {

	  tic("householder_denseqr");

		long m = A.size1();
		long n = A.size2();

		bool allocedStair = false;
		if (Stair == NULL) {
		  allocedStair = true;
			Stair = new long[n];
			for(int j=0; j<n; j++)
				Stair[j] = m;
		}

		tic("householder_denseqr: row->col");
		// convert from row major to column major
		ublas::matrix<double, ublas::column_major> Acolwise(A);
		double *a = Acolwise.data().begin();
    toc("householder_denseqr: row->col");

    tic("householder_denseqr: denseqr_front");
		long npiv = min(m,n);
		double tol = -1;	long ntol = -1; // no tolerance is used
		long fchunk = m < 32 ? m : 32;
		char Rdead[npiv];  memset(Rdead, 0, sizeof(char)*npiv);
		double Tau[n];
		long b = min(fchunk, min(n, m));
		double W[b*(n+b)];
		double wscale = 0;
		double wssq = 0;

//		cholmod_common cc;
//		cholmod_l_start(&cc);

		// todo: do something with the rank
		long rank = DenseQR(m, n, npiv, tol, ntol, fchunk,
				a, Stair, Rdead, Tau, W, &wscale, &wssq);
    toc("householder_denseqr: denseqr_front");

//#ifndef NDEBUG
		for(long j=0; j<npiv; ++j)
		  if(Rdead[j]) {
		    cout << "In householder_denseqr, aborting because some columns were found to be\n"
		        "numerically linearly-dependent and we cannot handle this case yet." << endl;
		    print(A, "The matrix being factored was\n");
		    ublas::matrix_range<ublas::matrix<double,ublas::column_major> > Acolsub(
		        ublas::project(Acolwise, ublas::range(0, min(m,n)), ublas::range(0,n)));
		    print(Matrix(ublas::triangular_adaptor<typeof(Acolsub), ublas::upper>(Acolsub)), "and the result was\n");
		    cout << "The following columns are \"dead\":";
		    for(long k=0; k<npiv; ++k)
		      if(Rdead[k]) cout << " " << k;
		    cout << endl;
		    exit(1);
		  }
//#endif

    tic("householder_denseqr: col->row");
		long k0 = 0;
		long j0;
		int k;
		memset(A.data().begin(), 0, m*n*sizeof(double));
		for(long j=0; j<n; j++, k0+=m) {
			k = k0;
			j0 = min(j+1,m);
			for(int i=0; i<j0; i++, k++)
				A(i,j) = a[k];
		}

//    ublas::matrix_range<ublas::matrix<double,ublas::column_major> > Acolsub(
//        ublas::project(Acolwise, ublas::range(0, min(m,n)), ublas::range(0,n)));
//    ublas::matrix_range<Matrix> Asub(ublas::project(A, ublas::range(0, min(m,n)), ublas::range(0,n)));
//		ublas::noalias(Asub) = ublas::triangular_adaptor<typeof(Acolsub), ublas::upper>(Acolsub);

    toc("householder_denseqr: col->row");

//		cholmod_l_finish(&cc);

		if(allocedStair) delete[] Stair;

		toc("householder_denseqr");
	}

	void householder_denseqr_colmajor(ublas::matrix<double, ublas::column_major>& A, long *Stair) {
    tic("householder_denseqr");

    long m = A.size1();
    long n = A.size2();

    assert(Stair != NULL);

    tic("householder_denseqr: denseqr_front");
    long npiv = min(m,n);
    double tol = -1;  long ntol = -1; // no tolerance is used
    long fchunk = m < 32 ? m : 32;
    char Rdead[npiv];  memset(Rdead, 0, sizeof(char)*npiv);
    double Tau[n];
    long b = min(fchunk, min(n, m));
    double W[b*(n+b)];
    double wscale = 0;
    double wssq = 0;

//    cholmod_common cc;
//    cholmod_l_start(&cc);

    // todo: do something with the rank
    long rank = DenseQR(m, n, npiv, tol, ntol, fchunk,
        A.data().begin(), Stair, Rdead, Tau, W, &wscale, &wssq);
    toc("householder_denseqr: denseqr_front");

//#ifndef NDEBUG
    for(long j=0; j<npiv; ++j)
      if(Rdead[j]) {
        cout << "In householder_denseqr, aborting because some columns were found to be\n"
            "numerically linearly-dependent and we cannot handle this case yet." << endl;
        print(A, "The matrix being factored was\n");
        ublas::matrix_range<ublas::matrix<double,ublas::column_major> > Acolsub(
            ublas::project(A, ublas::range(0, min(m,n)), ublas::range(0,n)));
        print(Matrix(ublas::triangular_adaptor<typeof(Acolsub), ublas::upper>(Acolsub)), "and the result was\n");
        cout << "The following columns are \"dead\":";
        for(long k=0; k<npiv; ++k)
          if(Rdead[k]) cout << " " << k;
        cout << endl;
        exit(1);
      }
//#endif

//    cholmod_l_finish(&cc);

    toc("householder_denseqr");

	}

} // namespace gtsam
#endif
