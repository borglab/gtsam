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

#include <string.h> // for memcpy and memset

using namespace std;
//namespace ublas = boost::numeric::ublas;

//#ifdef GT_USE_LAPACK
//namespace gtsam {
//
//	/* ************************************************************************* */
//	int* MakeStairs(Matrix& A) {
//
//		const int m = A.rows();
//		const int n = A.cols();
//		int* Stair = new int[n];
//
//		// record the starting pointer of each row
//		double* a[m];
//		list<int> remained;
//		a[0] = A.data();
//		for(int i=0; i<m-1; i++) {
//			a[i+1] = a[i] + n;
//			remained.push_back(i);
//		}
//		remained.push_back(m-1);
//
//		// reorder the rows
//		int j;
//		vector<int> sorted;
//		list<int>::iterator itRemained;
//		for(j = 0; j < n; ) {
//			// remove the non-zero rows in the current column
//			for(itRemained = remained.begin(); itRemained!=remained.end(); ) {
//				if (*(a[*itRemained]) != 0) {
//					sorted.push_back(*itRemained);
//					itRemained = remained.erase(itRemained);
//				} else {
//					a[*itRemained]++;
//					itRemained ++;
//				}
//			}
//
//			// record the stair number
//			Stair[j++] = m - remained.size();
//
//			if(remained.empty()) break;
//		}
//
//		// all the remained columns have maximum stair
//		for (; j<n; j++)
//			Stair[j] = m;
//
//		// copy the new row
//		Matrix A_new = zeros(m,n);
//		int offset[m];
//		offset[0] = 0;
//		for(int i=1; i<m; i++)
//			offset[i] = offset[i-1] +n;
//		vector<int>::const_iterator itSorted;
//		double* ptr1 = A.data();
//		double* ptr2 = A_new.data();
//		int row = 0, sizeOfRow = n * sizeof(double);
//		for(itSorted=sorted.begin(); itSorted!=sorted.end(); itSorted++, row++)
//			memcpy(ptr2+offset[row], ptr1+offset[*itSorted], sizeOfRow);
//
//		A = A_new;
//
//		return Stair;
//	}
//
//	void printColumnMajor(const double* a, const int m, const int n) {
//		for(int i=0; i<m; i++) {
//			for(int j=0; j<n; j++)
//				cout << a[j*m+i] << "\t";
//			cout << endl;
//		}
//	}
//
//	/* ************************************************************************* */
//	void householder_denseqr(Matrix &A, int* Stair) {
//
//	  tic("householder_denseqr");
//
//		int m = A.rows();
//		int n = A.cols();
//
//		bool allocedStair = false;
//		if (Stair == NULL) {
//		  allocedStair = true;
//			Stair = new int[n];
//			for(int j=0; j<n; j++)
//				Stair[j] = m;
//		}
//
//		tic("householder_denseqr: row->col");
//		// convert from row major to column major
//		MatrixColMajor Acolwise(A);
//		double *a = Acolwise.data();
//    toc("householder_denseqr: row->col");
//
//    tic("householder_denseqr: denseqr_front");
//		int npiv = min(m,n);
//		int b = min(min(m,n),32);
//    double W[b*(n+b)];
//		DenseQR(m, n, npiv, a, Stair, W);
//    toc("householder_denseqr: denseqr_front");
//
//    tic("householder_denseqr: col->row");
//		int k0 = 0;
//		int j0;
//		int k;
//		memset(A.data(), 0, m*n*sizeof(double));
//		for(int j=0; j<n; j++, k0+=m) {
//			k = k0;
//			j0 = min(j+1,m);
//			for(int i=0; i<j0; i++, k++)
//				A(i,j) = a[k];
//		}
//
//    toc("householder_denseqr: col->row");
//
//
//		if(allocedStair) delete[] Stair;
//
//		toc("householder_denseqr");
//	}
//
//	void householder_denseqr_colmajor(MatrixColMajor& A, int *Stair) {
//    int m = A.rows();
//    int n = A.cols();
//
//    assert(Stair != NULL);
//
//    tic(1, "householder_denseqr");
//    int npiv = min(m,n);
//		int b = min(min(m,n),32);
//    double W[b*(n+b)];
//    DenseQR(m, n, npiv, A.data(), Stair, W);
//    toc(1, "householder_denseqr");
//	}
//
//} // namespace gtsam
//#endif
