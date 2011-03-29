/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactor.cpp
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Richard Roberts, Christian Potthast
 */

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	GaussianFactor::GaussianFactor(const GaussianConditional& c) :
		IndexFactor(c) {
	}

}
