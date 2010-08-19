/*
 * Tensor4.h
 * @brief Rank 4 tensors based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#pragma once
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/** Rank 3 Tensor */
	template<int N1, int N2, int N3, int N4>
	class Tensor4 {

	private:

		Tensor3<N1, N2, N3> T[N4];

	public:

		/** default constructor */
		Tensor4() {
		}

		double operator()(int i, int j, int k, int l) const {
			return T[l](i, j, k);
		}

	}; // Tensor4

} // namespace tensors
