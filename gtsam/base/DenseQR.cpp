/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DenseQR.cpp
 *
 *   Created on: Oct 19, 2010
 *       Author: nikai
 *  Description: Dense QR, inspired by Tim Davis's dense solver
 */

#include <math.h>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <fstream>

#include "DenseQR.h"

//#define DEBUG_MEMORY

// all the lapack functions we need here
extern "C" {
void dlarft_ (char *direct, char *storev, int *n, int *k, double *V, int *ldv, double *Tau, double *T, int *ldt) ;
void dlarfb_ (char *side, char *trans, char *direct, char *storev, int *m, int *n, int *k, double *V, int *ldv, double *T, int *ldt, double *C, int *ldc, double *Work, int *ldwork) ;
void dlarfg_ (int *n, double *alpha, double *X, int *incx, double *tau) ;
void dlarf_ (char *side, int *m, int *n, double *V, int *incv, double *tau, double *C, int *ldc, double *Work) ;
}

using namespace std;

namespace gtsam {

	static int one = 1;
	static char left = 'L' ;
	static char direct = 'F';
	static char storev = 'C';
	static char trans = 'T';

	/* ************************************************************************* */
  // check NaN in the input matrix
  void CheckNaN(int m, int n, double *A, const char* msg) {
    bool hasNaN = false;
    for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
        if (isnan(A[j*m+i]))
          throw std::invalid_argument(msg);
		}
  }

	/* ************************************************************************* */
  // check Inf in the input matrix
  void CheckInf(int m, int n, double *A, const char* msg) {
    bool hasNaN = false;
    for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
        if (isinf(A[j*m+i]))
          throw std::invalid_argument(msg);
		}
  }

	/* ************************************************************************* */
  // remove NaN in the input matrix
  void RemoveNaN(int m, int n, double *A) {
    bool hasNaN = false;
    for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
        if (isnan(A[j*m+i])) 
          A[j*m+i] = 0;
		}
  }

	/* ************************************************************************* */
  // check NaN in the input matrix
  bool HasNaN(int m, int n, double *A) {
    bool hasNaN = false;
    for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
        if (isnan(A[j*m+i]))
          return true;
		}
    return false;
  }

	/* ************************************************************************* */
  // check Inf in the input matrix
  bool HasInf(int m, int n, double *A) {
    bool hasNaN = false;
    for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
        if (isinf(A[j*m+i]))
          return true;
		}
    return false;
  }

	/* ************************************************************************* */
	/**
	 * the wrapper for LAPACK dlarft_ and dlarfb_ function
	 */
	inline void dlarftb_wrap(int m, int n, int k, int ldc, int ldv, double *V, double *Tau, double *C, double *W,
			int* numPendingHHs, int* numZeros)
	{
      if (m <= 0 || n <= 0 || k <= 0) return ;

			if (m < k) throw std::invalid_argument("dlarftb_wrap: m < k!");
			dlarft_(&direct, &storev, &m, &k, V, &ldv, Tau, W, &k) ;
			dlarfb_(&left, &trans, &direct, &storev, &m, &n, &k, V, &ldv,
					W, &k, C, &ldc, W + k*k, &n);
			*numPendingHHs = 0;
			*numZeros = 0;
	}

	/* ************************************************************************* */
	/**
	 * check whether there is enough work left
	 */
	inline bool NotEnoughLeft(const int numRowLeft, const int numColLeft, const int sizeBlock) {
		return numRowLeft * (numColLeft - (sizeBlock + 4)) < 5000 || numRowLeft <= sizeBlock / 2
				|| sizeBlock <= 1;
	}

	/* ************************************************************************* */
  // write input to the disk for debugging
  void WriteInput(int m, int n, int numPivotColumns, double *A, int *stairs) {
    ofstream fs;
    fs.open ("denseqr.txt", ios::out | ios::trunc);
    fs << m << " " << n << " " <<  numPivotColumns << endl;
		for(int i=0; i<m; i++) {
			for(int j=0; j<n; j++)
				fs << A[j*m+i] << "\t";
			fs << endl;
		}
    fs.close();
  }

	/* ************************************************************************* */
	void DenseQR(int m, int n, int numPivotColumns, double *A, int *stairs, double *workspace) {

    // WriteInput(m, n, numPivotColumns, A, stairs);
    // CheckNaN(m, n, A, "DenseQR: the input matrix has NaN");
    // CheckInf(m, n, A, "DenseQR: the input matrix has Inf");

		if (A == NULL)         throw std::invalid_argument("DenseQR: A == NULL!");
		if (stairs == NULL)    throw std::invalid_argument("DenseQR: stairs == NULL!");
		if (workspace == NULL) throw std::invalid_argument("DenseQR: W == NULL!");
		if (m < 0)             throw std::invalid_argument("DenseQR: m < 0");
		if (n < 0)             throw std::invalid_argument("DenseQR: n < 0");
		if (numPivotColumns < 0 || numPivotColumns > n)
			throw std::invalid_argument("DenseQR: numPivotColumns < 0l || numPivotColumns > n");

		double tau, Tau[n]; // the scalar in Householder
		int row1stHH = 0, numGoodHHs = 0, numPendingHHs = 0;
		int colPendingHHStart = 0, colPendingHHEnd = 0;
		double *vectorHH = A;
		int numZeros = 0;
		int sizeBlock = m < 32 ? m : 32;
		int stairStartLast = 0, stairStart = 0, minSizeBlock = max(4l, sizeBlock/4l);
		int m1, n1;

		// go through all the columns
		for (int col = 0; col < n; col++) {
			// early termination because of non-full rank
			if (numGoodHHs >= m) {
				if (col < numPivotColumns) throw std::runtime_error("DenseQR: some columns were found to be linear independent!");
				return;
			}

			// if there is a enough large staircase change, apply Householder
			stairStartLast = stairStart;
			stairStart = max(numGoodHHs + 1, stairs[col]);
      stairs[col] = stairStart;
			numZeros += numPendingHHs * (stairStart - stairStartLast);
			if (numPendingHHs >= minSizeBlock && colPendingHHEnd < n &&
					numZeros > max(16, ((numPendingHHs * (numPendingHHs + 1)) / 2 + numPendingHHs * (stairStart - row1stHH - numPendingHHs)) / 2)) {
#ifdef DEBUG_MEMORY
				if (row1stHH >= m) throw std::runtime_error("DenseQR: row1stHH >= m");
				if (colPendingHHEnd >= n) throw std::runtime_error("DenseQR: colPendingHHEnd >= n");
#endif
        dlarftb_wrap(stairStartLast - row1stHH, n - colPendingHHEnd, numPendingHHs, m, m,
                     vectorHH, Tau + colPendingHHStart, &A[row1stHH+colPendingHHEnd*m], workspace, &numPendingHHs, &numZeros);
			}

			// compute Householder for the current column
			int n_ = stairStart - numGoodHHs;
			double *X = A + numGoodHHs + col*m;
			dlarfg_(&n_, X, X + 1, &one, &tau);
      Tau[col] = tau;
			if (!numPendingHHs) {
				row1stHH = numGoodHHs;
				vectorHH = A + row1stHH + col*m;
        colPendingHHStart = col;
#ifdef DEBUG_MEMORY
				if (row1stHH+col*m >= m*n) throw std::runtime_error("DenseQR: row1stHH+col*m >= m*n");
#endif
				colPendingHHEnd = NotEnoughLeft(m - row1stHH, n - col, sizeBlock) ? n : min(n, col + sizeBlock); // if not enough work left, go to unblocked mode
			}
			numPendingHHs++;

			// apply Householder reflector
			m1 = stairStart - numGoodHHs;
			n1 = colPendingHHEnd - col - 1;
			if (m1 > 0 && n1 > 0) {
				double *A1 = &A[numGoodHHs+col*m], *C1 = A1 + m, v0 = *A1;
				*A1 = 1 ;
				dlarf_ (&left, &m1, &n1, A1, &one, &tau, C1, &m, workspace) ;
				*A1 = v0;
			}
			numGoodHHs++;

			if ((numGoodHHs == m || col + 1 == colPendingHHEnd) && colPendingHHEnd < n) {
#ifdef DEBUG_MEMORY
				if (row1stHH >= m) throw std::runtime_error("DenseQR: row1stHH >= m");
				if (colPendingHHEnd >= n) throw std::runtime_error("DenseQR: colPendingHHEnd >= n");
#endif
        dlarftb_wrap(stairStart - row1stHH, n - colPendingHHEnd, numPendingHHs, m, m,
						vectorHH, Tau + colPendingHHStart, &A[row1stHH+colPendingHHEnd*m], workspace, &numPendingHHs, &numZeros);
			}
		}

    // CheckNaN(m, n, A, "DenseQR: the output matrix has NaN");
    // CheckInf(m, n, A, "DenseQR: the input matrix has Inf");

	}
} // namespace gtsam
