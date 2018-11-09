/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastList.h
 * @brief   A thin wrapper around std::list that uses a custom allocator.
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @date    Oct 22, 2010
 */

#pragma once

#include <gtsam/base/FastDefaultAllocator.h>
#include <list>

namespace gtsam {
/**
 * FastList is a type alias to a std::list with a custom memory allocator.
 * The particular allocator depends on GTSAM's cmake configuration.
 * @addtogroup base
 */
template <typename T>
using FastList =
    std::list<T, typename internal::FastDefaultAllocator<T>::type>;

}  // namespace gtsam
