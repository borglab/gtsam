/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridGaussianFactor.h
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/hybrid/HybridFactor.h>

namespace gtsam {

class HybridGaussianFactor : public HybridFactor {
 public:
  using Base = HybridFactor;

  GaussianFactor::shared_ptr inner;

  // Implicit conversion from a shared ptr of GF
  HybridGaussianFactor(GaussianFactor::shared_ptr other);

  // Forwarding constructor from concrete JacobianFactor
  HybridGaussianFactor(JacobianFactor &&jf);

 public:
  virtual bool equals(const HybridFactor& lf, double tol) const override;
};
}
