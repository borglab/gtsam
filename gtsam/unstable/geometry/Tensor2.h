/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Tensor2.h
 * @brief Rank 2 Tensor based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * @date Feb 10, 2010
 * @author Frank Dellaert
 */

#pragma once
#include <gtsam2/geometry/tensors.h>

namespace tensors {

/**
 * Rank 2 Tensor
 * @ingroup tensors
 * \nosubgrouping
 */
template<int N1, int N2>
class Tensor2 {
protected:
	Tensor1<N1> T[N2]; ///< Storage

public:

	/// @name Standard Constructors
	/// @{

	/** default constructor */
	Tensor2() {
	}

	/// construct from data - expressed in row major form
	Tensor2(const double data[N2][N1]) {
		for (int j = 0; j < N2; j++)
			T[j] = Tensor1<N1> (data[j]);
	}

	/** construct from expression */
	template<class A, char I, char J>
	Tensor2(const Tensor2Expression<A, Index<N1, I> , Index<N2, J> >& a) {
		for (int j = 0; j < N2; j++)
			T[j] = a(j);
	}

	/// @}
	/// @name Standard Interface
	/// @{

	/** dimension - TODO: is this right for anything other than 3x3? */
	size_t dim() const {return N1 * N2;}

	/// const element access
	const double & operator()(int i, int j) const {
		return T[j](i);
	}

	/// element access
	double & operator()(int i, int j) {
		return T[j](i);
	}

	/** convert to expression */
	template<char I, char J> Tensor2Expression<Tensor2, Index<N1, I> , Index<
			N2, J> > operator()(Index<N1, I> i, Index<N2, J> j) const {
		return Tensor2Expression<Tensor2, Index<N1, I> , Index<N2, J> > (*this);
	}

  /// @}

};

} // namespace tensors

