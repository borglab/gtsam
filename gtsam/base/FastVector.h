/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastVector.h
 * @brief   Type alias for std::vector (previously used a custom allocator).
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @date    Feb 9, 2011
 */

#pragma once

#include <vector>

namespace gtsam {

/**
 * FastVector is a type alias for std::vector.
 * Previously used a custom memory allocator, but now uses the default allocator.
 * This type alias is maintained for backward compatibility.
 * 
 * To use a custom allocator with vectors in GTSAM, you can either:
 * 1. Use std::vector<T, CustomAllocator<T>> directly where needed
 * 2. Define your own type alias with the allocator you need
 * 
 * @ingroup base
 */
template <typename T>
using FastVector = std::vector<T>;

}  // namespace gtsam
