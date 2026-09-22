/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FactorErrorPredicate.h
 * @brief Predicate used to filter factor-graph error output.
 */

#pragma once

#include <cstddef>
#include <functional>

namespace gtsam {

class Factor;

/// Predicate used to select factor errors for graph diagnostics.
using FactorErrorPredicate =
    std::function<bool(const Factor*, double, std::size_t)>;

}  // namespace gtsam
