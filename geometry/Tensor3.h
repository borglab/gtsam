/*
 * Tensor3.h
 * @brief Rank 3 tensors based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 10, 2010
 * @author: Frank Dellaert
 */

#pragma once
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/** Rank 3 Tensor */
	template<int N1, int N2, int N3>
	class Tensor3 {
		Tensor2<N1, N2> T[N3];

	public:

		/** default constructor */
		Tensor3() {
		}

		/** construct from data */
		Tensor3(const double data[N3][N2][N1]) {
			for (int k = 0; k < N3; k++)
				T[k] = data[k];
		}

		/** construct from expression */
		template<class A, char I, char J, char K>
		Tensor3(const Tensor3Expression<A, Index<N1, I> , Index<N2, J> , Index<N3,
				K> >& a) {
			for (int k = 0; k < N3; k++)
				T[k] = a(k);
		}

		double operator()(int i, int j, int k) const {
			return T[k](i, j);
		}

		/** convert to expression */
		template<char I, char J, char K> Tensor3Expression<Tensor3, Index<N1, I> ,
				Index<N2, J> , Index<N3, K> > operator()(const Index<N1, I>& i,
				const Index<N2, J>& j, const Index<N3, K>& k) {
			return Tensor3Expression<Tensor3, Index<N1, I> , Index<N2, J> , Index<N3,
					K> > (*this);
		}

		/** convert to expression */
		template<char I, char J, char K> Tensor3Expression<const Tensor3, Index<N1, I> ,
				Index<N2, J> , Index<N3, K> > operator()(const Index<N1, I>& i,
				const Index<N2, J>& j, const Index<N3, K>& k) const {
			return Tensor3Expression<const Tensor3, Index<N1, I> , Index<N2, J> , Index<N3,
					K> > (*this);
		}
	}; // Tensor3

	/** Rank 3 permutation tensor */
	struct Eta3 {

		/** calculate value. TODO: wasteful to actually use this */
		double operator()(int i, int j, int k) const {
			return ((j - i) * (k - i) * (k - j)) / 2;
		}

		/** create expression */
		template<char I, char J, char K> Tensor3Expression<Eta3, Index<3, I> ,
				Index<3, J> , Index<3, K> > operator()(const Index<3, I>& i,
				const Index<3, J>& j, const Index<3, K>& k) const {
			return Tensor3Expression<Eta3, Index<3, I> , Index<3, J> , Index<3, K> > (
					*this);
		}

	}; // Eta

} // namespace tensors
