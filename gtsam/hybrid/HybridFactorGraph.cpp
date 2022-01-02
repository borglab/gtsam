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
  for (auto&& dcFactor : dcGraph_) {
    // If dcFactor is a DCGaussianMixtureFactor, we don't linearize.
    if (boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      linearized_DC_factors.push_back(dcFactor);
    } else {
      auto linearizedDCFactor = dcFactor->linearize(continuousValues);
      linearized_DC_factors.push_back(linearizedDCFactor);
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    gaussianFactorGraph->push_back(gaussianFactor);
  }

  // Construct new linearized HybridFactorGraph
  return HybridFactorGraph({}, discreteGraph_, linearized_DC_factors,
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

/// Define adding a GaussianFactor to a sum.
using Sum = DCGaussianMixtureFactor::Sum;
static Sum& operator+=(Sum& sum, const GaussianFactor::shared_ptr& factor) {
  using Y = GaussianFactorGraph::shared_ptr;
  std::function<Y(const Y&)> add = [&factor](const Y& graph) {
    auto result = boost::make_shared<GaussianFactorGraph>(*graph);
    result->push_back(factor);
    return result;
  };
  sum = sum.apply(add);
  return sum;
}

Sum HybridFactorGraph::sum() const {
  if (nrNonlinearFactors()) {
    throw std::runtime_error(
        "HybridFactorGraph::sum cannot handle NonlinearFactors.");
  }

  if (nrDiscreteFactors()) {
    throw std::runtime_error(
        "HybridFactorGraph::sum cannot handle DiscreteFactors.");
  }

  // TODO: take all factors, which *must* be all DCGaussianMixtureFactors or
  // GaussianFactors, "add" them (which might involve decision-trees of
  // different structure, and creating a dummy decision tree for Gaussians).

  // SUM: "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      sum += *mixtureFactor;
    } else {
      throw std::runtime_error(
          "HybridFactorGraph::sum can only handleDCGaussianMixtureFactors.");
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    sum += gaussianFactor;
  }
  return sum;
}

/// The function type that does a single elimination step on a variable.
std::pair<DCConditional::shared_ptr, boost::shared_ptr<Factor>> EliminateHybrid(
    const HybridFactorGraph& factors, const Ordering& ordering) {
  auto sum = factors.sum();

  // If there are no DC factors, this would be appropriate:
  auto result = EliminatePreferCholesky(factors.gaussianGraph(), ordering);
  boost::shared_ptr<GaussianConditional> gc = result.first;
  boost::shared_ptr<GaussianFactor> gf = result.second;

  // Create a DCConditional...
  auto conditional = boost::make_shared<DCConditional>();

  // Create a resulting DCGaussianMixture on the separator.
  /// auto factor = TODO ...
  return {conditional, nullptr};
}

}  // namespace gtsam
