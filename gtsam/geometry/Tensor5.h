/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Tensor5.h
 * @brief Rank 5 tensors based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#pragma once
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/** Rank 3 Tensor */
	template<int N1, int N2, int N3, int N4, int N5>
	class Tensor5 {

	private:

		Tensor4<N1, N2, N3, N4> T[N5]; ///< Storage

	public:

		/** default constructor */
		Tensor5() {
		}

		/** construct from expression */
		template<class A, char I, char J, char K, char L, char M>
		Tensor5(const Tensor5Expression<A, Index<N1, I> , Index<N2, J> , Index<N3,
				K> , Index<N4, L> , Index<N5, M> >& a) {
			for (int m = 0; m < N5; m++)
				T[m] = a(m);
		}

		/// element access
		double operator()(int i, int j, int k, int l, int m) const {
			return T[m](i, j, k, l);
		}

		/** convert to expression */
		template<char I, char J, char K, char L, char M> Tensor5Expression<Tensor5,
				Index<N1, I> , Index<N2, J> , Index<N3, K> , Index<N4, L> ,
				Index<N5, M> > operator()(Index<N1, I> i, Index<N2, J> j,
				Index<N3, K> k, Index<N4, L> l, Index<N5, M> m) {
			return Tensor5Expression<Tensor5, Index<N1, I> , Index<N2, J> , Index<N3,
					K> , Index<N4, L> , Index<N5, M> > (*this);
		}
	}; // Tensor5

} // namespace tensors
