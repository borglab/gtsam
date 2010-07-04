/*
 * SPQRUtil.cpp
 *
 *   Created on: Jul 1, 2010
 *       Author: nikai
 *  Description: the utility functions for SPQR
 */

#include <map>
#include "SPQRUtil.h"

using namespace std;

#ifdef GT_USE_LAPACK
namespace gtsam {

	/* ************************************************************************* */
	long* MakeStairs(Matrix& A) {

		const long m = A.size1();
		const long n = A.size2();

		// record the starting pointer of each row
		double* a[m];
		a[0] = A.data().begin();
		for(int i=1; i<m; i++)
			a[i] = a[i-1] + n;

		// go through each column from left to right
		int j;
		int i0, i1, i2, tmpi;
		long* Stair = new long[n];
		int sizeOfRow;
		double tmp[n], *row1, *row2;
		double tmpd;
		for(j = 0; j < min(m,n); ) {

			i0 = (j==0) ? 0 : Stair[j-1];

			// find all the rows with leading zeros in the current column
			vector<int> i_zeros, i_nonzeros, i_all;
			map<int, int> i2vi;
			for(int i = i0; i < m; i++) {
				i_all.push_back(i);
				i2vi.insert(make_pair(i, i-i0));
				if (*(a[i]) == 0)
					i_zeros.push_back(i);
				else
					i_nonzeros.push_back(i);
			}

			// resort the rows from i_all to i_target
			vector<int>& i_target = i_nonzeros;
			i_target.insert(i_nonzeros.end(), i_zeros.begin(), i_zeros.end());
			sizeOfRow = (n - j) * sizeof(double);
			for (int vi=0; vi<m-i0; vi++) {
				i1 = i_all[vi];
				i2 = i_target[vi];
				if (i1 != i2) {
					row1 = a[vi + i0];
					tmpi = i2vi[i2];
					row2 = a[i2vi[i2] + i0];
					memcpy(tmp,  row1, sizeOfRow);
					memcpy(row1, row2, sizeOfRow);
					memcpy(row2, tmp,  sizeOfRow);
					std::swap(i_all[vi], i_all[tmpi]);
					std::swap(i2vi[i1], i2vi[i2]);
				}
			}

			// record the stair number
			Stair[j++] = m - i_zeros.size();
			if (i_zeros.empty()) break;

			// right shift the pointers
			for(int i=0; i<m; i++) a[i]++;
		}

		// all the remained columns have maximum stair
		for (; j<n; j++)
			Stair[j] = m;

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
	void householder_spqr(Matrix &A, long* Stair) {
		long m = A.size1();
		long n = A.size2();

		if (Stair == NULL) {
			Stair = new long[n];
			for(int j=0; j<n; j++)
				Stair[j] = m;
		}

		// convert from row major to column major
		double a[m*n]; int k = 0;
		for(int j=0; j<n; j++)
			for(int i=0; i<m; i++, k++)
				a[k] = A(i,j);

		long npiv = min(m,n);
		double tol = -1;	long ntol = -1; // no tolerance is used
		long fchunk = m < 32 ? m : 32;
		char Rdead[npiv];
		double Tau[n];
		long b = min(fchunk, min(n, m));
		double W[b*(n+b)];
		double wscale = 0;
		double wssq = 0;

		cholmod_common cc;
		cholmod_l_start(&cc);

		long rank = spqr_front<double>(m, n, npiv, tol, ntol, fchunk,
				a, Stair, Rdead, Tau, W, &wscale, &wssq, &cc);

		long k0 = 0;
		long j0;
		memset(A.data().begin(), 0, m*n*sizeof(double));
		for(long j=0; j<n; j++, k0+=m) {
			k = k0;
			j0 = min(j+1,m);
			for(int i=0; i<j0; i++, k++)
				A(i,j) = a[k];
		}

		delete []Stair;
		cholmod_l_finish(&cc);
	}

} // namespace gtsam
#endif
