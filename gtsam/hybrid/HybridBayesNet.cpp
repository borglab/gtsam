//
// Created by Fan Jiang on 1/24/22.
//

#include <gtsam/hybrid/HybridBayesNet.h>

namespace gtsam {

GaussianMixture::shared_ptr HybridBayesNet::atGaussian(size_t i) const {
  return boost::dynamic_pointer_cast<GaussianMixture>(factors_.at(i));
}

DiscreteConditional::shared_ptr HybridBayesNet::atDiscrete(size_t i) const {
  return boost::dynamic_pointer_cast<DiscreteConditional>(factors_.at(i));
}

GaussianBayesNet HybridBayesNet::operator()(
    const DiscreteValues& assignment) const {
  GaussianBayesNet gbn;
  for (size_t idx = 0; idx < size(); idx++) {
    GaussianMixture gm = *this->atGaussian(idx);
    gbn.push_back(gm(assignment));
  }
  return gbn;
}

}  // namespace gtsam
