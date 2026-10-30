/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file AugmentedPoint.cpp
 * @brief Explicit instantiations of commonly used augmented point dimensions.
 * @author Oluwatodimu Afolayan
 */

#include <gtsam/geometry/AugmentedPoint.h>

namespace gtsam {

// Explicit instantiations for 1D C++ tests and 2D/3D Python and MATLAB
// wrappers.
template class AugmentedPoint<1>;
template class AugmentedPoint<2>;
template class AugmentedPoint<3>;

}  // namespace gtsam
