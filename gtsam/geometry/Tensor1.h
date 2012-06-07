/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Tensor1.h
 * @brief Rank 1 tensors based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 10, 2010
 * @author Frank Dellaert
 */

#pragma once
#include <gtsam/geometry/tensors.h>

namespace tensors {

	/**
	 * A rank 1 tensor. Actually stores data.
	 * @addtogroup tensors
	 * \nosubgrouping
	 */
	template<int N>
	class Tensor1 {
		double T[N]; ///< Storage

	public:

		/// @name Standard Constructors
		/// @{

		/** default constructor */
		Tensor1() {
		}

		/** construct from data */
		Tensor1(const double* data) {
			for (int i = 0; i < N; i++)
				T[i] = data[i];
		}

		/** construct from expression */
		template<class A, char I>
		Tensor1(const Tensor1Expression<A, Index<N, I> >& a) {
			for (int i = 0; i < N; i++)
				T[i] = a(i);
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/** return data */
		inline int dim() const {
			return N;
		}

		/** return data */
		inline const double& operator()(int i) const {
			return T[i];
		}

		/** return data */
		inline double& operator()(int i) {
			return T[i];
		}

		/// return an expression associated with an index
		template<char I> Tensor1Expression<Tensor1, Index<N, I> > operator()(
				Index<N, I> index) const {
			return Tensor1Expression<Tensor1, Index<N, I> >(*this);
		}

		/// @}

	};
// Tensor1

}// namespace tensors
