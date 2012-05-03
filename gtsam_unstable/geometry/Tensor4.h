/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Tensor4.h
 * @brief Rank 4 tensors based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 12, 2010
 * @author Frank Dellaert
 */

#pragma once
#include <gtsam_unstable/geometry/tensors.h>

namespace tensors {

	/**
	 * Rank 4 Tensor
	 * @ingroup tensors
	 * \nosubgrouping
	 */
	template<int N1, int N2, int N3, int N4>
	class Tensor4 {

	private:

		Tensor3<N1, N2, N3> T[N4]; ///< Storage

	public:

		/// @name Standard Constructors
		/// @{

		/** default constructor */
		Tensor4() {
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/// element access
		double operator()(int i, int j, int k, int l) const {
			return T[l](i, j, k);
		}

		/// @}

	}; // Tensor4

} // namespace tensors
