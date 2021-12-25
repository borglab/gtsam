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

#include <gtsam/hybrid/HybridFactorGraph.h>

namespace gtsam {

HybridFactorGraph::HybridFactorGraph() {}

void HybridFactorGraph::push_nonlinear(
    const boost::shared_ptr<gtsam::NonlinearFactor>& nonlinearFactor) {
  nonlinearGraph_.push_back(nonlinearFactor);
}

void HybridFactorGraph::push_discrete(
    const DiscreteFactor::shared_ptr& discreteFactor) {
  discreteGraph_.push_back(discreteFactor);
}

void HybridFactorGraph::push_dc(const DCFactor::shared_ptr& dcFactor) {
  dcGraph_.push_back(dcFactor);
}

void HybridFactorGraph::print(const std::string& str,
                              const gtsam::KeyFormatter& keyFormatter) const {
  std::string prefix = str.empty() ? str : str + ": ";
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

HybridFactorGraph HybridFactorGraph::linearize(
    const Values& continuousValues) const {
  // linearize the continuous factors
  auto gaussian_factor_graph = nonlinearGraph_.linearize(continuousValues);

  // linearize the DCFactors
  DCFactorGraph linearized_DC_factors;
  for (DCFactor::shared_ptr factor : dcGraph_) {
    auto dcgf = factor->linearize(continuousValues);
    linearized_DC_factors.push_back(dcgf);
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

}  // namespace gtsam
