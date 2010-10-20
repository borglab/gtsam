/*
 * DenseQR.cpp
 *
 *   Created on: Oct 19, 2010
 *       Author: nikai
 *  Description: Dense QR, inspired by Tim Davis's dense solver
 */

#include <cassert>
#include <math.h>
#include <algorithm>

#include "DenseQR.h"

// all the lapack functions we need here
extern "C" {
void dlarft_ (char *direct, char *storev, int *n, int *k, double *V, int *ldv, double *Tau, double *T, int *ldt) ;
void dlarfb_ (char *side, char *trans, char *direct, char *storev, int *m, int *n, int *k, double *V, int *ldv, double *T, int *ldt, double *C, int *ldc, double *Work, int *ldwork) ;
void dlarfg_ (int *n, double *alpha, double *X, int *incx, double *tau) ;
void dlarf_ (char *side, int *m, int *n, double *V, int *incv, double *tau, double *C, int *ldc, double *Work) ;
}

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	/**
	 * LARF applies a real elementary reflector H to a real m by n matrix
	 * C, from either the left or the right. H is represented in the form
	 */
	void dlarf_wrap(long m, long n, long ldc, double *V, double tau, double *C, double *W)
	{
		static char left = 'L' ;
		double vsave ;
		if (m <= 0 || n <= 0) return ;
		vsave = V [0] ;     // temporarily restore unit diagonal of V
		V [0] = 1 ;
		int m_ = m, n_ = n, ldc_ = ldc, one = 1 ;
		dlarf_ (&left, &m_, &n_, V, &one, &tau, C, &ldc_, W) ;
		V [0] = vsave ;     // restore V [0]
	}

	/* ************************************************************************* */
	void dlarftb_wrap(long m, long n, long k, long ldc, long ldv, double *V, double *Tau, double *C, double *W)
	{
			static char direct = 'F';
			static char storev = 'C';
			static char side = 'L';
			static char trans = 'T';
			if (m <= 0 || n <= 0 || k <= 0)	return ;

			double *T, *Work ;
			T = W ;             // triangular k-by-k matrix for block reflector
			Work = W + k*k ;    // workspace of size n*k or m*k for larfb

			// construct and apply the k-by-k upper triangular matrix T
			// larft and larfb are always used "Forward" and "Columnwise"
			assert (m >= k) ;
			int m_ = m, n_ = n, k_ = k, ldv_ = ldv, ldc_ = ldc;
			dlarft_(&direct, &storev, &m_, &k_, V, &ldv_, Tau, T, &k_) ;
			// Left, Transpose, Forward, Columwise:
			dlarfb_(&side, &trans, &direct, &storev, &m_, &n_, &k_, V, &ldv_,
					T, &k_, C, &ldc_, Work, &n_);

	}

	/* ************************************************************************* */
	long DenseQR(long m, long n, long npiv, double tol, long ntol, long fchunk,
			double *F, long *Stair, char *Rdead, double *Tau, double *W, double *wscale, double *wssq) {
		double tau, wk, *V;
		long k, t, g, g1, nv, k1, k2, i, t0, vzeros, mleft, nleft, vsize, minchunk, rank ;

		assert (F != NULL) ;
		assert (Stair != NULL) ;
		assert (Rdead != NULL) ;
		assert (Tau != NULL) ;
		assert (W != NULL) ;
		assert (m >= 0 && n >= 0) ;

		npiv = max (0l, npiv) ;  // npiv must be between 0 and n
		npiv = min (n, npiv) ;
		g1 = 0 ;                // row index of first queued-up Householder
		k1 = 0 ;                // pending Householders are in F (g1:t, k1:k2-1)
		k2 = 0 ;
		V = F ;                 // Householder vectors start here
		g = 0 ;                 // number of good Householders found
		nv = 0 ;                // number of Householder reflections queued up
		vzeros = 0 ;            // number of explicit zeros in queued-up H's
		t = 0 ;                 // staircase of current column
		fchunk = max (fchunk, 1l) ;
		minchunk = max (4l, fchunk/4l) ;
		rank = min (m,npiv) ;
		ntol = min (ntol, npiv) ;

		for (k = 0; k < n; k++) {
			t0 = t; // t0 = staircase of column k-1
			t = Stair[k]; // t = staircase of this column k

			if (g >= m) {
				for (; k < npiv; k++) {
					Rdead[k] = 1;
					Stair[k] = 0;
					Tau[k] = 0;
				}
				for (; k < n; k++) {
					Stair[k] = m;
					Tau[k] = 0;
				}
				assert (nv == 0);
				return (rank);
			}

			t = max(g + 1, t);
			Stair[k] = t;

			vzeros += nv * (t - t0);
			if (nv >= minchunk) {
				vsize = (nv * (nv + 1)) / 2 + nv * (t - g1 - nv);
				if (vzeros > max(16l, vsize / 2)) {
					dlarftb_wrap(t0 - g1, n - k2, nv, m, m, V, // F (g1:t-1, k1:k1+nv-1)
							&Tau[k1], &F[g1+k2*m], W);
					nv = 0;
					vzeros = 0;
				}
			}

			// find a Householder reflection that reduces column k
			int n_ = t - g, one = 1;
			double *X = &F[g+k*m];
			dlarfg_(&n_, X, X + 1, &one, &tau);

			// check to see if the kth column is OK
			if (k < ntol && (wk = fabs(F[g+k*m])) <= tol) {
				if (wk != 0) {
					if ((*wscale) == 0) {
						(*wssq) = 1;
					}
					if ((*wscale) < wk) {
						double rr = (*wscale) / wk;
						(*wssq) = 1 + (*wssq) * rr * rr;
						(*wscale) = wk;
					} else {
						double rr = wk / (*wscale);
						(*wssq) += rr * rr;
					}
				}

				// zero out F (g:m-1,k) and flag it as dead
				for (i = g; i < m; i++)
					F[i+k*m] = 0;
				Stair[k] = 0;
				Tau[k] = 0;
				Rdead[k] = 1;

				// apply pending block of Householder reflections
				if (nv > 0) {
					dlarftb_wrap(t0 - g1, n - k2, nv, m, m, V, &Tau[k1], &F[g1+k2*m], W);
					nv = 0; // clear queued-up Householder reflections
					vzeros = 0;
				}
			} else {
				// one more good pivot column found
				Tau[k] = tau;
				if (nv == 0) {
					g1 = g;
					k1 = k;
					k2 = min(n, k + fchunk);
					V = &F[g1+k1*m];

					// check for switch to unblocked code
					mleft = m - g1; // number of rows left
					nleft = n - k1; // number of columns left
					if (mleft * (nleft - (fchunk + 4)) < 5000 || mleft <= fchunk / 2
							|| fchunk <= 1)
						k2 = n;
				}
				nv++; // one more pending update; V is F (g1:t-1, k1:k1+nv-1)

				// apply the kth Householder reflection to the current panel
				dlarf_wrap(t - g, k2 - k - 1, m, &F[g+k*m], tau, &F[g+(k+1)*m], W);

				g++; // one more pivot found

				if (k == k2 - 1 || g == m) {
					dlarftb_wrap(t - g1, n - k2, nv, m, m, V, &Tau[k1], &F[g1+(k2*m)], W);
					nv = 0; // clear queued-up Householder reflections
					vzeros = 0;
				}
			}

			if (k == npiv - 1) rank = g;
		}

		return rank;
	}
}
