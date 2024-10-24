/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2024, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * @file TransferFactor.h
 * @brief TransferFactor class
 * @author Frank Dellaert
 * @date October 24, 2024
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/inference/EdgeKey.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Binary factor in the context of Structure from Motion (SfM).
 * It is used to transfer points between different views based on the
 * fundamental matrices between these views. The factor computes the error
 * between the transferred points `pi` and `pj`, and the actual point `pk` in
 * the target view. Jacobians are done using numerical differentiation.
 */
template <typename F>
class TransferFactor : public NoiseModelFactorN<F, F> {
  EdgeKey key1_, key2_;  ///< the two EdgeKeys
  Point2 pi, pj, pk;     ///< The points in the three views

 public:
  /**
   * @brief Constructor for the TransferFactor class.
   *
   * Uses EdgeKeys to determine how to use the two fundamental matrix unknowns
   * F1 and F2, to transfer points pi and pj to the third view, and minimize the
   * difference with pk.
   *
   * The edge keys must represent valid edges for the transfer operation,
   * specifically one of the following configurations:
   * - (i, k) and (j, k)
   * - (i, k) and (k, j)
   * - (k, i) and (j, k)
   * - (k, i) and (k, j)
   *
   * @param key1 First EdgeKey specifying F1: (i, k) or (k, i).
   * @param key2 Second EdgeKey specifying F2: (j, k) or (k, j).
   * @param pi The point in the first view (i).
   * @param pj The point in the second view (j).
   * @param pk The point in the third (and transfer target) view (k).
   * @param model An optional SharedNoiseModel that defines the noise model
   *              for this factor. Defaults to nullptr.
   */
  TransferFactor(EdgeKey key1, EdgeKey key2, const Point2& pi, const Point2& pj,
                 const Point2& pk, const SharedNoiseModel& model = nullptr)
      : NoiseModelFactorN<F, F>(model, key1, key2),
        key1_(key1),
        key2_(key2),
        pi(pi),
        pj(pj),
        pk(pk) {}

  // Create Matrix3 objects based on EdgeKey configurations
  std::pair<Matrix3, Matrix3> getMatrices(const F& F1, const F& F2) const {
    // Fill Fki and Fkj based on EdgeKey configurations
    if (key1_.i() == key2_.i()) {
      return {F1.matrix(), F2.matrix()};
    } else if (key1_.i() == key2_.j()) {
      return {F1.matrix(), F2.matrix().transpose()};
    } else if (key1_.j() == key2_.i()) {
      return {F1.matrix().transpose(), F2.matrix()};
    } else if (key1_.j() == key2_.j()) {
      return {F1.matrix().transpose(), F2.matrix().transpose()};
    } else {
      throw std::runtime_error(
          "TransferFactor: invalid EdgeKey configuration.");
    }
  }

  /// vector of errors returns 2D vector
  Vector evaluateError(const F& F1, const F& F2,
                       OptionalMatrixType H1 = nullptr,
                       OptionalMatrixType H2 = nullptr) const override {
    std::function<Point2(F, F)> transfer = [&](const F& F1, const F& F2) {
      auto [Fki, Fkj] = getMatrices(F1, F2);
      return F::transfer(Fki, pi, Fkj, pj);
    };
    if (H1) *H1 = numericalDerivative21<Point2, F, F>(transfer, F1, F2);
    if (H2) *H2 = numericalDerivative22<Point2, F, F>(transfer, F1, F2);
    return transfer(F1, F2) - pk;
  }
};

}  // namespace gtsam
