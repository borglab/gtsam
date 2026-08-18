/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TernaryJacobianFactor.h
 * @brief Compatibility alias for a three-key FixedJacobianFactor.
 */

#pragma once

#include <gtsam/linear/FixedJacobianFactor.h>

namespace gtsam {

/** A three-key JacobianFactor with compile-time block dimensions. */
template <int M, int N1, int N2, int N3>
using TernaryJacobianFactor = FixedJacobianFactor<M, N1, N2, N3>;

}  // namespace gtsam
