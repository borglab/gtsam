/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastVector.h
 * @brief   A thin wrapper around std::vector that uses a custom allocator.
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @date    Feb 9, 2011
 */

#pragma once

#include <gtsam/base/FastDefaultAllocator.h>
#include <vector>

namespace gtsam {

/**
 * FastVector is a type alias to a std::vector with a custom memory allocator.
 * The particular allocator depends on GTSAM's cmake configuration.
 * @ingroup base
 */
template <typename T>
using FastVector =
    std::vector<T, typename internal::FastDefaultVectorAllocator<T>::type>;

}  // namespace gtsam
