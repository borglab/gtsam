/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file make_shared.h
 * @brief make_shared trampoline function to ensure proper alignment
 * @author Fan Jiang
 */

#pragma once

#include <gtsam/base/types.h>

#include <Eigen/Core>

#include <boost/make_shared.hpp>

#include <type_traits>

namespace gtsam {
  /// An shorthand alias for accessing the \::type inside std::enable_if that can be used in a template directly
  template<bool B, class T = void>
  using enable_if_t = typename std::enable_if<B, T>::type;
}

namespace gtsam {

  /**
   * Add our own `make_shared` as a layer of wrapping on `boost::make_shared`
   * This solves the problem with the stock `make_shared` that custom alignment is not respected, causing SEGFAULTs
   * at runtime, which is notoriously hard to debug.
   *
   * Explanation
   * ===============
   * The template `needs_eigen_aligned_allocator<T>::value` will evaluate to `std::true_type` if the type alias
   * `_eigen_aligned_allocator_trait = void` is present in a class, which is automatically added by the
   * `GTSAM_MAKE_ALIGNED_OPERATOR_NEW` macro.
   *
   * This function declaration will only be taken when the above condition is true, so if some object does not need to
   * be aligned, `gtsam::make_shared` will fall back to the next definition, which is a simple wrapper for
   * `boost::make_shared`.
   *
   * @tparam T The type of object being constructed
   * @tparam Args Type of the arguments of the constructor
   * @param args Arguments of the constructor
   * @return The object created as a boost::shared_ptr<T>
   */
  template<typename T, typename ... Args>
  gtsam::enable_if_t<needs_eigen_aligned_allocator<T>::value, boost::shared_ptr<T>> make_shared(Args &&... args) {
    return boost::allocate_shared<T>(Eigen::aligned_allocator<T>(), std::forward<Args>(args)...);
  }

  /// Fall back to the boost version if no need for alignment
  template<typename T, typename ... Args>
  gtsam::enable_if_t<!needs_eigen_aligned_allocator<T>::value, boost::shared_ptr<T>> make_shared(Args &&... args) {
    return boost::make_shared<T>(std::forward<Args>(args)...);
  }

}
