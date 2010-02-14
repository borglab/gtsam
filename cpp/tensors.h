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
		enum { dim = Dim };
	};

} // namespace tensors

// Expression templates
#include "Tensor1Expression.h"
#include "Tensor2Expression.h"
#include "Tensor3Expression.h"
// Tensor4 not needed so far
#include "Tensor5Expression.h"

// Actual tensor classes
#include "Tensor1.h"
#include "Tensor2.h"
#include "Tensor3.h"
#include "Tensor4.h"
#include "Tensor5.h"


