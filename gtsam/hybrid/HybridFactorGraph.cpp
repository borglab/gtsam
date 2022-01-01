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

#include <boost/make_shared.hpp>

namespace gtsam {

// Instantiate base classes
// template class FactorGraph<Factor>;
template class EliminateableFactorGraph<HybridFactorGraph>;

void HybridFactorGraph::print(const std::string& str,
                              const gtsam::KeyFormatter& keyFormatter) const {
  std::string prefix = str.empty() ? str : str + ": ";
  std::cout << prefix << "size: " << size() << std::endl;
  nonlinearGraph_.print(prefix + "NonlinearFactorGraph", keyFormatter);
  discreteGraph_.print(prefix + "DiscreteFactorGraph", keyFormatter);
  dcGraph_.print(prefix + "DCFactorGraph", keyFormatter);
  gaussianGraph_.print(prefix + "GaussianGraph", keyFormatter);
}

gtsam::FastSet<gtsam::Key> HybridFactorGraph::keys() const {
  gtsam::FastSet<gtsam::Key> keys;
  // Combine keys from all the internal graphs
  keys.merge(nonlinearGraph_.keys());
  keys.merge(discreteGraph_.keys());
  keys.merge(dcGraph_.keys());
  return keys;
}

const gtsam::NonlinearFactorGraph& HybridFactorGraph::nonlinearGraph() const {
  return nonlinearGraph_;
}

const gtsam::DiscreteFactorGraph& HybridFactorGraph::discreteGraph() const {
  return discreteGraph_;
}

const GaussianFactorGraph& HybridFactorGraph::gaussianGraph() const {
  return gaussianGraph_;
}

HybridFactorGraph HybridFactorGraph::linearize(
    const Values& continuousValues) const {
  // linearize the continuous factors
  auto gaussian_factor_graph = nonlinearGraph_.linearize(continuousValues);

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
    gaussian_factor_graph->push_back(factor);
  }

  // Construct new linearized HybridFactorGraph
  HybridFactorGraph linearizedGraph(this->nonlinearGraph_, this->discreteGraph_,
                                    linearized_DC_factors,
                                    *gaussian_factor_graph);
  return linearizedGraph;
}

const DCFactorGraph& HybridFactorGraph::dcGraph() const { return dcGraph_; }

bool HybridFactorGraph::empty() const {
  return nonlinearGraph_.empty() && discreteGraph_.empty() && dcGraph_.empty();
}

bool HybridFactorGraph::equals(const HybridFactorGraph& other,
                               double tol) const {
  return nonlinearGraph_.equals(other.nonlinearGraph_, tol) &&
         discreteGraph_.equals(other.discreteGraph_, tol) &&
         dcGraph_.equals(other.dcGraph_, tol);
}

size_t HybridFactorGraph::size() const {
  return nonlinearGraph_.size() + discreteGraph_.size() + dcGraph_.size();
}

size_t HybridFactorGraph::size_nonlinear() const {
  return nonlinearGraph_.size();
}

size_t HybridFactorGraph::size_discrete() const {
  return discreteGraph_.size();
}

size_t HybridFactorGraph::size_dc() const { return dcGraph_.size(); }

void HybridFactorGraph::clear() {
  nonlinearGraph_.resize(0);
  discreteGraph_.resize(0);
  dcGraph_.resize(0);
}

/// The function type that does a single elimination step on a variable.
std::pair<DCConditional::shared_ptr, boost::shared_ptr<Factor>> EliminateHybrid(
    const HybridFactorGraph& factors, const Ordering& ordering) {
  // We are getting a number of DCMixtureFactors on a set of continuous
  // variables. They might all have different discrete keys. For every
  // possible combination of the discrete keys, we need a GaussianConditional.
  // for (const auto& factor : factors) {
  //   if (auto p = boost::dynamic_pointer_cast<const
  //   DCGaussianMixtureFactor>(
  //           factor)) {
  //     GTSAM_PRINT(*p);
  //   };
  // }

  std::cout << "HybridEliminate" << std::endl;
  GTSAM_PRINT(factors);

  // Create a DCConditional...
  auto conditional = boost::make_shared<DCConditional>();

  // Create a resulting DCGaussianMixture on the separator.
  /// auto factor = TODO ...
  return {conditional, nullptr};
}

}  // namespace gtsam
