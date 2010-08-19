/*
 * SPQRUtil.cpp
 *
 *   Created on: Jul 1, 2010
 *       Author: nikai
 *  Description: the utility functions for SPQR
 */

#include <map>
#include <gtsam/base/SPQRUtil.h>

using namespace std;

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
