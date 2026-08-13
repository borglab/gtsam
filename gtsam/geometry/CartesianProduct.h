/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CartesianProduct.h
 * @brief Compatibility name for the direct product of two Lie groups.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

/**
 * Direct Cartesian product of two Lie groups.
 *
 * This descriptive alias retains the API introduced for continuous trajectory
 * control points while sharing the fully tested ProductLieGroup
 * implementation.
 */
template <class A, class B>
using CartesianProduct = ProductLieGroup<A, B>;

}  // namespace gtsam
