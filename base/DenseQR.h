/*
 * DenseQR.h
 *
 *   Created on: Oct 19, 2010
 *       Author: nikai
 *  Description: Dense QR, inspired by Tim Davis's dense solver
 */

#pragma once

namespace gtsam {
	long DenseQR(
			long m,              // F is m-by-n with leading dimension m
			long n,
			long npiv,           // number of pivot columns
			double tol,          // a column is flagged as dead if its norm is <= tol
			long ntol,           // apply tol only to first ntol pivot columns
			long fchunk,         // block size for compact WY Householder reflections,
													 // treated as 1 if fchunk <= 1

			// input/output
			double *F,           // frontal matrix F of size m-by-n
			long *Stair,         // size n, entries F (Stair[k]:m-1, k) are all zero,
													 // and remain zero on output.
			char *Rdead,         // size npiv; all zero on input.  If k is dead,
													 // Rdead [k] is set to 1

			// output, not defined on input
			double *Tau,         // size n, Householder coefficients

			// workspace, undefined on input and output
			double *W,           // size b*(n+b), where b = min (fchunk,n,m)

			// input/output
			double *wscale,
			double *wssq
	);
}
