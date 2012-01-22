/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianDensity.cpp
 * @brief   A Gaussian Density
 * @author  Frank Dellaert
 * @date    Jan 21, 2012
 */

#include <gtsam/linear/GaussianDensity.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	Vector GaussianDensity::mean() const {
		// Solve for mean
		VectorValues x;
		Index k = firstFrontalKey();
		// a VectorValues that only has a value for k: cannot be printed if k<>0
		x.insert(k, Vector(sigmas_.size()));
		rhs(x);
		solveInPlace(x);
		return x[k];
	}

	/* ************************************************************************* */
	Matrix GaussianDensity::information() const {
		return computeInformation();
	}

	/* ************************************************************************* */
	Matrix GaussianDensity::covariance() const {
		return inverse(information());
	}

} // gtsam
