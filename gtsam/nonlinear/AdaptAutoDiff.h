/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AdaptAutoDiff.h
 * @date October 22, 2014
 * @author Frank Dellaert
 * @brief Adaptor for Ceres style auto-differentiated functions
 */

#pragma once

#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/3rdparty/ceres/autodiff.h>

namespace gtsam {

/**
 * The AdaptAutoDiff class uses ceres-style autodiff to adapt a ceres-style
 * Function evaluation, i.e., a function FUNCTOR that defines an operator
 *   template<typename T> bool operator()(const T* const, const T* const, T*
 * predicted) const;
 * For now only binary operators are supported.
 */
template <typename FUNCTOR, int M, int N1, int N2>
class AdaptAutoDiff {
  typedef Eigen::Matrix<double, M, N1, Eigen::RowMajor> RowMajor1;
  typedef Eigen::Matrix<double, M, N2, Eigen::RowMajor> RowMajor2;

  typedef Eigen::Matrix<double, M, 1> VectorT;
  typedef Eigen::Matrix<double, N1, 1> Vector1;
  typedef Eigen::Matrix<double, N2, 1> Vector2;

  FUNCTOR f;

 public:
  VectorT operator()(const Vector1& v1, const Vector2& v2,
                     OptionalJacobian<M, N1> H1 = {},
                     OptionalJacobian<M, N2> H2 = {}) {
    using ceres::internal::AutoDiff;

    bool success;
    VectorT result;

    if (H1 || H2) {
      // Get derivatives with AutoDiff
      const double* parameters[] = {v1.data(), v2.data()};
      double rowMajor1[M * N1] = {}, rowMajor2[M * N2] = {};  // on the stack
      double* jacobians[] = {rowMajor1, rowMajor2};
      success = AutoDiff<FUNCTOR, double, N1, N2>::Differentiate(
          f, parameters, M, result.data(), jacobians);

      // Convert from row-major to columnn-major
      // TODO: if this is a bottleneck (probably not!) fix Autodiff to be
      // Column-Major
      if (H1) *H1 = Eigen::Map<RowMajor1>(rowMajor1);
      if (H2) *H2 = Eigen::Map<RowMajor2>(rowMajor2);

    } else {
      // Apply the mapping, to get result
      success = f(v1.data(), v2.data(), result.data());
    }
    if (!success)
      throw std::runtime_error(
          "AdaptAutoDiff: function call resulted in failure");
    return result;
  }
};

}  // namespace gtsam
