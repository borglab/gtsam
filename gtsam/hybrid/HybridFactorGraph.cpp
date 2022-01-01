/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.cpp
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

// Instantiate base classes
// template class FactorGraph<Factor>;
template class EliminateableFactorGraph<HybridFactorGraph>;

void HybridFactorGraph::print(const std::string& str,
                              const gtsam::KeyFormatter& keyFormatter) const {
  std::string prefix = str.empty() ? str : str + ".";
  std::cout << prefix << "size: " << size() << std::endl;
  nonlinearGraph_.print(prefix + "NonlinearFactorGraph", keyFormatter);
  discreteGraph_.print(prefix + "DiscreteFactorGraph", keyFormatter);
  dcGraph_.print(prefix + "DCFactorGraph", keyFormatter);
  gaussianGraph_.print(prefix + "GaussianGraph", keyFormatter);
}

HybridFactorGraph HybridFactorGraph::linearize(
    const Values& continuousValues) const {
  // linearize the continuous factors
  auto gaussianFactorGraph = nonlinearGraph_.linearize(continuousValues);

  // linearize the DCFactors
  DCFactorGraph linearized_DC_factors;
  for (DCFactor::shared_ptr factor : dcGraph_) {
    // If factor is a DCGaussianMixtureFactor, we don't linearize.
    auto dcgm = dynamic_cast<const DCGaussianMixtureFactor*>(factor.get());
    if (dcgm) {
      linearized_DC_factors.push_back(factor);

    } else {
      auto dcgf = factor->linearize(continuousValues);
      linearized_DC_factors.push_back(dcgf);
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& factor : this->gaussianGraph()) {
    gaussianFactorGraph->push_back(factor);
  }

  // Construct new linearized HybridFactorGraph
  return HybridFactorGraph({}, this->discreteGraph_, linearized_DC_factors,
                           *gaussianFactorGraph);
}

bool HybridFactorGraph::equals(const HybridFactorGraph& other,
                               double tol) const {
  return Base::equals(other, tol) &&
         nonlinearGraph_.equals(other.nonlinearGraph_, tol) &&
         discreteGraph_.equals(other.discreteGraph_, tol) &&
         dcGraph_.equals(other.dcGraph_, tol) &&
         gaussianGraph_.equals(other.gaussianGraph_, tol);
}

void HybridFactorGraph::clear() {
  nonlinearGraph_.resize(0);
  discreteGraph_.resize(0);
  dcGraph_.resize(0);
  gaussianGraph_.resize(0);
}

/// The function type that does a single elimination step on a variable.
std::pair<DCConditional::shared_ptr, boost::shared_ptr<Factor>> EliminateHybrid(
    const HybridFactorGraph& factors, const Ordering& ordering) {
  std::cout << "HybridEliminate" << std::endl;
  std::cout << "factors.size():" << factors.size() << std::endl;
  GTSAM_PRINT(factors);

  // Create a DCConditional...
  auto conditional = boost::make_shared<DCConditional>();

  // TODO: take all factors, which *must* be all DCGaussianMixtureFactors or
  // GaussianFactors, "add" them (which might involve decision-trees of
  // different structure, and creating a dummy decision tree for Gaussians).

  // If there are no DC factors, this is appropriate:
  auto result = EliminatePreferCholesky(factors.gaussianGraph(), ordering);

  // Create a resulting DCGaussianMixture on the separator.
  /// auto factor = TODO ...
  return {conditional, nullptr};
}

}  // namespace gtsam
