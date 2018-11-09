/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author set)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FastSet.h
 * @brief   A thin wrapper around std::set that uses a custom allocator.
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @date    Oct 22, 2010
 */

#pragma once

#include <gtsam/base/FastDefaultAllocator.h>

#include <functional>
#include <set>

namespace gtsam {
/**
 * FastSet is a type alias to a std::set with a custom memory allocator.
 * The particular allocator depends on GTSAM's cmake configuration.
 * @addtogroup base
 */
template <class T, class _Compare = std::less<T>>
using FastSet =
    std::set<T, _Compare, typename internal::FastDefaultAllocator<T>::type>;

// /** Handy 'exists' function */
// bool exists(const VALUE& e) const {
//   return this->find(e) != this->end();
// }

// /** Print to implement Testable: pretty basic */
// void print(const std::string& str = "") const {
//   for (typename Base::const_iterator it = this->begin(); it != this->end();
//   ++it) traits<VALUE>::Print(*it, str);
// }

// /** Check for equality within tolerance to implement Testable */
// bool equals(const FastSet<VALUE>& other, double tol = 1e-9) const {
//   typename Base::const_iterator it1 = this->begin(), it2 = other.begin();
//   while (it1 != this->end()) {
//     if (it2 == other.end() || !traits<VALUE>::Equals(*it2, *it2, tol))
//     return false;
//     ++it1;
//     ++it2;
//   }
//   return true;
// }

// /** insert another set: handy for MATLAB access */
// void merge(const FastSet& other) {
//   Base::insert(other.begin(), other.end());
// }

}  // namespace gtsam
