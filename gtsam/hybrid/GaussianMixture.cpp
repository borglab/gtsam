/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixture.cpp
 * @brief  Discrete-continuous conditional density
 * @author Frank Dellaert
 * @date   December 2021
 */

#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/Conditional.h>

namespace gtsam {

// Instantiate base class
template class Conditional<DCGaussianMixtureFactor, GaussianMixture>;

}  // namespace gtsam
