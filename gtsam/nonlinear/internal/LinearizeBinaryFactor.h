/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearizeBinaryFactor.h
 * @brief Internal fixed-size linearization helper for binary nonlinear factors.
 */

#pragma once

#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam {
namespace internal {

/**
 * Linearize a two-key noise-model factor to a fixed-size binary Jacobian
 * factor. Dynamically sized instantiations retain the generic implementation.
 *
 * This helper is intended for factors whose existing `unwhitenedError()` path
 * returns dynamic Jacobian matrices. Factors such as `GeneralSFMFactor` that
 * already compute directly into fixed-size Jacobians retain their specialized
 * linearization to avoid introducing these dynamic temporaries.
 */
template <int M, int N1, int N2>
std::shared_ptr<GaussianFactor> linearizeBinaryFactor(
    const NoiseModelFactor& factor, const Values& values) {
  if constexpr (M == Eigen::Dynamic || N1 == Eigen::Dynamic ||
                N2 == Eigen::Dynamic) {
    return factor.NoiseModelFactor::linearize(values);
  } else {
    static_assert(M > 0 && N1 > 0 && N2 > 0,
                  "Binary factor dimensions must be positive");

    if (!factor.active(values)) return std::shared_ptr<JacobianFactor>();

    std::vector<Matrix> jacobians(2);
    Vector b = -factor.unwhitenedError(values, jacobians);
    const SharedNoiseModel& noiseModel = factor.noiseModel();
    if (noiseModel && static_cast<size_t>(b.size()) != noiseModel->dim()) {
      throw std::invalid_argument(
          "NoiseModelFactor: NoiseModel has dimension " +
          std::to_string(noiseModel->dim()) + " instead of " +
          std::to_string(b.size()) + ".");
    }
    if (b.size() != M || jacobians[0].rows() != M ||
        jacobians[0].cols() != N1 || jacobians[1].rows() != M ||
        jacobians[1].cols() != N2) {
      throw std::invalid_argument(
          "linearizeBinaryFactor: error or Jacobian dimension mismatch");
    }

    if (noiseModel) {
      noiseModel->WhitenSystem(jacobians[0], jacobians[1], b);
    }

    SharedDiagonal linearModel;
    if (noiseModel && noiseModel->isConstrained()) {
      linearModel =
          std::static_pointer_cast<noiseModel::Constrained>(noiseModel)->unit();
    }

    const Eigen::Matrix<double, M, N1> A1 = jacobians[0];
    const Eigen::Matrix<double, M, N2> A2 = jacobians[1];
    const Eigen::Matrix<double, M, 1> fixedB = b;
    return std::make_shared<BinaryJacobianFactor<M, N1, N2>>(
        factor.keys()[0], A1, factor.keys()[1], A2, fixedB, linearModel);
  }
}

}  // namespace internal
}  // namespace gtsam
