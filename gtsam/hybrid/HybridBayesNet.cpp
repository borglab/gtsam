//
// Created by Fan Jiang on 1/24/22.
//

#include <gtsam/hybrid/HybridBayesNet.h>

namespace gtsam {

GaussianMixture::shared_ptr HybridBayesNet::atGaussian(size_t i) {
  return boost::dynamic_pointer_cast<GaussianMixture>(factors_.at(i));
}

}