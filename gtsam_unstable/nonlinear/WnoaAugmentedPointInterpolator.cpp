
/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file WnoaAugmentedPointInterpolator.cpp
 * @brief Explicit interpolator instantiations for AugmentedPoint.
 * @author Oluwatodimu Afolayan
 */

#include <gtsam/nonlinear/WnoaInterpolator-inl.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/geometry/AugmentedPoint.h>

namespace gtsam {

template class GTSAM_UNSTABLE_EXPORT Interpolator<AugmentedPoint<2>>;
template class GTSAM_UNSTABLE_EXPORT Interpolator<AugmentedPoint<3>>;

}  // namespace gtsam
