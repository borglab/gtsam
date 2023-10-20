/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Utils.h
 * @brief   Utilities for the Chi Squared inverse and related operations.
 * @author  Varun Agrawal
 */

#pragma once

namespace gtsam {
namespace internal {

/// Template type for numeric limits
template <class T>
using LIM = std::numeric_limits<T>;

template <typename T>
using return_t =
    typename std::conditional<std::is_integral<T>::value, double, T>::type;

/// Get common type amongst all arguments
template <typename... T>
using common_t = typename std::common_type<T...>::type;

/// Helper template for finding common return type
template <typename... T>
using common_return_t = return_t<common_t<T...>>;

/// Check if integer is odd
constexpr bool is_odd(const long long int x) noexcept { return (x & 1U) != 0; }

/// Templated check for NaN
template <typename T>
constexpr bool is_nan(const T x) noexcept {
  return x != x;
}

}  // namespace internal
}  // namespace gtsam
