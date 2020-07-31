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

#include <boost/assign/list_of.hpp>

namespace gtsam {
/**
 * The ShonanGaugeFactor creates a constraint on a single SO(n) to avoid moving
 * in the stabilizer.
 *
 * Details: SO(n) contains the n*3 Stiefel matrices of orthogonal frames: we
 * take those to be the 3 columns on the left.
 * The n*n skew symmetric matrices associated with so(n) can be partitioned as
 * (Appendix B in the ECCV paper):
 * | [w] -K' |
 * |  K  [g] |
 * where w is the SO(3) space, K are the extra Stiefel diemnsions (wormhole!)
 * and (g)amma are extra dimensions in SO(n) that do not modify the cost
 * function. The latter corresponds to rotations SO(p-d), and so the first few
 * values of p-d for d==3 with their corresponding dimensionality are {0:0, 1:0,
 * 2:1, 3:3, ...}
 *
 * The ShonanGaugeFactor adds a unit Jacobian to these extra dimensions,
 * essentially restricting the optimization to the Stiefel manifold.
 */
class ShonanGaugeFactor : public NonlinearFactor {
  // Row dimension, eqaul to the dimensionality of SO(p-d)
  size_t rows_;

  /// Constant Jacobian
  boost::shared_ptr<JacobianFactor> jacobian_;

public:
  /// Construct from key for an SO(p) matrix, for base dimension d (2 or 3)
  ShonanGaugeFactor(Key key, size_t p, size_t d = 3)
      : NonlinearFactor(boost::assign::cref_list_of<1>(key)) {
    if (p < d) {
      throw std::invalid_argument("ShonanGaugeFactor must have p>=d.");
    }
    // Calculate dimensions
    size_t q = p - d;
    size_t P = SOn::Dimension(p); // dimensionality of SO(p)
    rows_ = SOn::Dimension(q);    // dimensionality of SO(q), the gauge

    // Create constant Jacobian as a rows_*P matrix: there are rows_ penalized
    // dimensions!
    Matrix A(rows_, P);
    A.setZero();
    size_t i = 0, j = 0, n = p - 1 - d;
    while (i < rows_) {
      A.block(i, j, n, n) = Matrix::Identity(n, n);
      i += n;
      j += n + d; // skip d columns
      n -= 1;
    }
    // TODO(frank): assign the right one in the right columns
    jacobian_ = boost::make_shared<JacobianFactor>(key, A, Vector::Zero(rows_));
  }

  /// Destructor
  virtual ~ShonanGaugeFactor() {}

  /// Calculate the error of the factor: always zero
  double error(const Values &c) const override { return 0; }

  /// get the dimension of the factor (number of rows on linearization)
  size_t dim() const override { return rows_; }

  /// linearize to a GaussianFactor
  boost::shared_ptr<GaussianFactor> linearize(const Values &c) const override {
    return jacobian_;
  }
};
// \ShonanGaugeFactor

} // namespace gtsam