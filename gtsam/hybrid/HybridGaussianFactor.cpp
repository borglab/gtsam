/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridGaussianFactor.cpp
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#include <gtsam/hybrid/HybridGaussianFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

HybridGaussianFactor::HybridGaussianFactor(GaussianFactor::shared_ptr other)
    : Base(other->keys()) {
  inner = other;
}

HybridGaussianFactor::HybridGaussianFactor(JacobianFactor &&jf)
    : Base(jf.keys()),
      inner(boost::make_shared<JacobianFactor>(std::move(jf))) {}

bool HybridGaussianFactor::equals(const HybridFactor &lf, double tol) const {
  return false;
}
void HybridGaussianFactor::print(const std::string &s,
                                 const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  inner->print("inner: ", formatter);
};

}  // namespace gtsam