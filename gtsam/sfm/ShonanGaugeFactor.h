/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanGaugeFactor.h
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Factor used in Shonan Averaging to clamp down gauge freedom
 */

#pragma once

#include <gtsam/geometry/SOn.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {
/**
 * The ShonanGaugeFactor creates a constraint on a single SO(n) to avoid moving
 * in the stabilizer.
 *
 * Details: SO(p) contains the p*3 Stiefel matrices of orthogonal frames: we
 * take those to be the 3 columns on the left.
 * The P*P skew-symmetric matrices associated with so(p) can be partitioned as
 * (Appendix B in the ECCV paper):
 * | [w] -K' |
 * |  K  [g] |
 * where w is the SO(3) space, K are the extra Stiefel diemnsions (wormhole!)
 * and (g)amma are extra dimensions in SO(p) that do not modify the cost
 * function. The latter corresponds to rotations SO(p-d), and so the first few
 * values of p-d for d==3 with their corresponding dimensionality are {0:0, 1:0,
 * 2:1, 3:3, ...}
 *
 * The ShonanGaugeFactor adds a unit Jacobian to these extra dimensions,
 * essentially restricting the optimization to the Stiefel manifold.
 */
class GTSAM_EXPORT ShonanGaugeFactor : public NonlinearFactor {
  // Row dimension, equal to the dimensionality of SO(p-d)
  size_t rows_;

  /// Constant Jacobian
  boost::shared_ptr<JacobianFactor> whitenedJacobian_;

public:
  /**
   * Construct from key for an SO(p) matrix, for base dimension d (2 or 3)
   * If parameter gamma is given, it acts as a precision = 1/sigma^2, and
   * the Jacobian will be multiplied with 1/sigma = sqrt(gamma).
   */
  ShonanGaugeFactor(Key key, size_t p, size_t d = 3,
                    boost::optional<double> gamma = boost::none)
      : NonlinearFactor(KeyVector{key}) {
    if (p < d) {
      throw std::invalid_argument("ShonanGaugeFactor must have p>=d.");
    }
    // Calculate dimensions
    size_t q = p - d;
    size_t P = SOn::Dimension(p); // dimensionality of SO(p)
    rows_ = SOn::Dimension(q);    // dimensionality of SO(q), the gauge

    // Create constant Jacobian as a rows_*P matrix: there are rows_ penalized
    // dimensions, but it is a bit tricky to find them among the P columns.
    // The key is to look at how skew-symmetric matrices are laid out in SOn.h:
    // the first tangent dimension will always be included, but beyond that we
    // have to be careful. We always need to skip the d top-rows of the skew-
    // symmetric matrix as they below to K, part of the Stiefel manifold.
    Matrix A(rows_, P);
    A.setZero();
    double invSigma = gamma ? std::sqrt(*gamma) : 1.0;
    size_t i = 0, j = 0, n = p - 1 - d;
    while (i < rows_) {
      A.block(i, j, n, n) = invSigma * Matrix::Identity(n, n);
      i += n;
      j += n + d; // skip d columns
      n -= 1;
    }
    // TODO(frank): assign the right one in the right columns
    whitenedJacobian_ =
        boost::make_shared<JacobianFactor>(key, A, Vector::Zero(rows_));
  }

  /// Destructor
  ~ShonanGaugeFactor() override {}

  /// Calculate the error of the factor: always zero
  double error(const Values &c) const override { return 0; }

  /// get the dimension of the factor (number of rows on linearization)
  size_t dim() const override { return rows_; }

  /// linearize to a GaussianFactor
  boost::shared_ptr<GaussianFactor> linearize(const Values &c) const override {
    return whitenedJacobian_;
  }
};
// \ShonanGaugeFactor

} // namespace gtsam