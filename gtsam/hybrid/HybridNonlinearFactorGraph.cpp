/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactorGraph.cpp
 * @brief  Nonlinear hybrid factor graph that uses type erasure
 * @author Varun Agrawal
 * @date   May 28, 2022
 */

#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
void HybridNonlinearFactorGraph::add(
    boost::shared_ptr<NonlinearFactor> factor) {
  FactorGraph::add(boost::make_shared<HybridNonlinearFactor>(factor));
}

/* ************************************************************************* */
void HybridNonlinearFactorGraph::add(boost::shared_ptr<DiscreteFactor> factor) {
  FactorGraph::add(boost::make_shared<HybridDiscreteFactor>(factor));
}

/* ************************************************************************* */
void HybridNonlinearFactorGraph::print(const std::string& s,
                                       const KeyFormatter& keyFormatter) const {
  // Base::print(str, keyFormatter);
  std::cout << (s.empty() ? "" : s + " ") << std::endl;
  std::cout << "size: " << size() << std::endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    std::stringstream ss;
    ss << "factor " << i << ": ";
    if (factors_[i]) {
      factors_[i]->print(ss.str(), keyFormatter);
      std::cout << std::endl;
    }
  }
}

/* ************************************************************************* */
HybridGaussianFactorGraph::shared_ptr HybridNonlinearFactorGraph::linearize(
    const Values& continuousValues) const {
  // create an empty linear FG
  auto linearFG = boost::make_shared<HybridGaussianFactorGraph>();

  linearFG->reserve(size());

  // linearize all hybrid factors
  for (auto&& factor : factors_) {
    // First check if it is a valid factor
    if (factor) {
      // Check if the factor is a hybrid factor.
      // It can be either a nonlinear MixtureFactor or a linear
      // GaussianMixtureFactor.
      if (factor->isHybrid()) {
        // Check if it is a nonlinear mixture factor
        if (auto nlmf = boost::dynamic_pointer_cast<MixtureFactor>(factor)) {
          linearFG->push_back(nlmf->linearize(continuousValues));
        } else {
          linearFG->push_back(factor);
        }

        // Now check if the factor is a continuous only factor.
      } else if (factor->isContinuous()) {
        // In this case, we check if factor->inner() is nonlinear since
        // HybridFactors wrap over continuous factors.
        auto nlhf = boost::dynamic_pointer_cast<HybridNonlinearFactor>(factor);
        if (auto nlf =
                boost::dynamic_pointer_cast<NonlinearFactor>(nlhf->inner())) {
          auto hgf = boost::make_shared<HybridGaussianFactor>(
              nlf->linearize(continuousValues));
          linearFG->push_back(hgf);
        } else {
          linearFG->push_back(factor);
        }
        // Finally if nothing else, we are discrete-only which doesn't need
        // lineariztion.
      } else {
        linearFG->push_back(factor);
      }

    } else {
      linearFG->push_back(GaussianFactor::shared_ptr());
    }
  }
  return linearFG;
}

}  // namespace gtsam
