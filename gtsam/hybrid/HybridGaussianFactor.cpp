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
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

/* ************************************************************************* */
HybridGaussianFactor::HybridGaussianFactor(
    const boost::shared_ptr<GaussianFactor> &ptr)
    : Base(ptr->keys()), inner_(ptr) {}

HybridGaussianFactor::HybridGaussianFactor(
    boost::shared_ptr<GaussianFactor> &&ptr)
    : Base(ptr->keys()), inner_(std::move(ptr)) {}

HybridGaussianFactor::HybridGaussianFactor(JacobianFactor &&jf)
    : Base(jf.keys()),
      inner_(boost::make_shared<JacobianFactor>(std::move(jf))) {}

HybridGaussianFactor::HybridGaussianFactor(HessianFactor &&hf)
    : Base(hf.keys()),
      inner_(boost::make_shared<HessianFactor>(std::move(hf))) {}

/* ************************************************************************* */
bool HybridGaussianFactor::equals(const HybridFactor &other, double tol) const {
  const This *e = dynamic_cast<const This *>(&other);
  if (e == nullptr) return false;
  if (!Base::equals(*e, tol)) return false;
  return inner_ ? (e->inner_ ? inner_->equals(*(e->inner_), tol) : false)
                : !(e->inner_);
}

/* ************************************************************************* */
void HybridGaussianFactor::print(const std::string &s,
                                 const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  if (inner_) {
    inner_->print("\n", formatter);
  } else {
    std::cout << "\nGaussian: nullptr" << std::endl;
  }
};

/* ************************************************************************ */
double HybridGaussianFactor::error(const HybridValues &values) const {
  return inner_->error(values.continuous());
}
/* ************************************************************************ */

}  // namespace gtsam
