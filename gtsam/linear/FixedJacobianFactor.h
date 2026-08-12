/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedJacobianFactor.h
 * @brief Compile-time machinery for constructing fixed-size Jacobian factors.
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/TernaryJacobianFactor.h>

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace gtsam {
namespace internal {

/** Compile-time residual and variable dimensions for a factor of any arity. */
template <typename OutputVec, typename... ValueTypes>
struct FixedSizeDimensions {
  inline constexpr static size_t arity = sizeof...(ValueTypes);
  inline constexpr static int M = traits<OutputVec>::dimension;
  inline constexpr static std::array<int, arity + 1> dimensions{
      M, traits<ValueTypes>::dimension...};
  template <size_t I>
  inline constexpr static int N = dimensions.at(I + 1);
  inline constexpr static bool allFixed =
      M != Eigen::Dynamic &&
      ((traits<ValueTypes>::dimension != Eigen::Dynamic) && ...);
};

/** Adapter from fixed dimensions to an available specialized Jacobian factor. */
template <int M, int... Ns>
struct FixedJacobianFactorFactory {
  inline constexpr static bool available = false;
};

template <int M, int N1, int N2>
struct FixedJacobianFactorFactory<M, N1, N2> {
  inline constexpr static bool available = true;

  static std::shared_ptr<GaussianFactor> create(
      const KeyVector& keys, const std::vector<Matrix>& jacobians,
      const Vector& b, const SharedDiagonal& model) {
    const Eigen::Matrix<double, M, N1> A1 = jacobians[0];
    const Eigen::Matrix<double, M, N2> A2 = jacobians[1];
    const Eigen::Matrix<double, M, 1> fixedB = b;
    return std::make_shared<BinaryJacobianFactor<M, N1, N2>>(
        keys[0], A1, keys[1], A2, fixedB, model);
  }
};

template <int M, int N1, int N2, int N3>
struct FixedJacobianFactorFactory<M, N1, N2, N3> {
  inline constexpr static bool available = true;

  static std::shared_ptr<GaussianFactor> create(
      const KeyVector& keys, const std::vector<Matrix>& jacobians,
      const Vector& b, const SharedDiagonal& model) {
    const Eigen::Matrix<double, M, N1> A1 = jacobians[0];
    const Eigen::Matrix<double, M, N2> A2 = jacobians[1];
    const Eigen::Matrix<double, M, N3> A3 = jacobians[2];
    const Eigen::Matrix<double, M, 1> fixedB = b;
    return std::make_shared<TernaryJacobianFactor<M, N1, N2, N3>>(
        keys[0], A1, keys[1], A2, keys[2], A3, fixedB, model);
  }
};

/** Check dynamic Jacobian blocks against their compile-time dimensions. */
template <typename Dimensions, size_t... Is>
bool dimensionsMatch(const std::vector<Matrix>& jacobians,
                     std::index_sequence<Is...>) {
  return ((jacobians[Is].rows() == Dimensions::M &&
           jacobians[Is].cols() == Dimensions::template N<Is>) &&
          ...);
}

}  // namespace internal
}  // namespace gtsam
