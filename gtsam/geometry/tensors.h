/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * tensors.h
 * @brief Tensor expression templates based on http://www.gps.caltech.edu/~walter/FTensor/FTensor.pdf
 * Created on: Feb 10, 2010
 * @author: Frank Dellaert
 */

#pragma once

namespace tensors {

	/** index */
	template<int Dim, char C> struct Index {
		static const int dim = Dim;
	};

} // namespace tensors

// Expression templates
#include <gtsam/geometry/Tensor1Expression.h>
#include <gtsam/geometry/Tensor2Expression.h>
#include <gtsam/geometry/Tensor3Expression.h>
// Tensor4 not needed so far
#include <gtsam/geometry/Tensor5Expression.h>

// Actual tensor classes
#include <gtsam/geometry/Tensor1.h>
#include <gtsam/geometry/Tensor2.h>
#include <gtsam/geometry/Tensor3.h>
#include <gtsam/geometry/Tensor4.h>
#include <gtsam/geometry/Tensor5.h>


