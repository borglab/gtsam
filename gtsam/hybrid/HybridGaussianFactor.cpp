//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/HybridGaussianFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

HybridGaussianFactor::HybridGaussianFactor(GaussianFactor::shared_ptr other) : Base(other->keys()) {
  inner = other;
}

HybridGaussianFactor::HybridGaussianFactor(JacobianFactor &&jf) : Base(jf.keys()),  inner(boost::make_shared<JacobianFactor>(std::move(jf))) {

}

bool HybridGaussianFactor::equals(const HybridFactor& lf, double tol) const {
  return false;
}
void HybridGaussianFactor::print(const std::string &s, const KeyFormatter &formatter) const {
  HybridFactor::print(s, formatter);
  inner->print("inner: ", formatter);
};

}