/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianHybridFactorGraph.cpp
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

// Instantiate base classes
// template class FactorGraph<Factor>;
template class EliminateableFactorGraph<GaussianHybridFactorGraph>;

void GaussianHybridFactorGraph::print(
    const string& str, const gtsam::KeyFormatter& keyFormatter) const {
  string prefix = str.empty() ? str : str + ".";
  cout << prefix << "size: " << size() << endl;
  discreteGraph_.print(prefix + "DiscreteFactorGraph", keyFormatter);
  dcGraph_.print(prefix + "DCFactorGraph", keyFormatter);
  gaussianGraph_.print(prefix + "GaussianGraph", keyFormatter);
}

bool GaussianHybridFactorGraph::equals(const GaussianHybridFactorGraph& other,
                                       double tol) const {
  return Base::equals(other, tol) &&
         discreteGraph_.equals(other.discreteGraph_, tol) &&
         dcGraph_.equals(other.dcGraph_, tol) &&
         gaussianGraph_.equals(other.gaussianGraph_, tol);
}

void GaussianHybridFactorGraph::clear() {
  discreteGraph_.resize(0);
  dcGraph_.resize(0);
  gaussianGraph_.resize(0);
}

/// Define adding a GaussianFactor to a sum.
using Sum = DCGaussianMixtureFactor::Sum;
static Sum& operator+=(Sum& sum, const GaussianFactor::shared_ptr& factor) {
  using Y = GaussianFactorGraph;
  auto add = [&factor](const Y& graph) {
    auto result = graph;
    result.push_back(factor);
    return result;
  };
  sum = sum.apply(add);
  return sum;
}

Sum GaussianHybridFactorGraph::sum() const {
  if (nrDiscreteFactors()) {
    throw runtime_error(
        "GaussianHybridFactorGraph::sum cannot handle DiscreteFactors.");
  }

  // "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      sum += *mixtureFactor;
    } else {
      throw runtime_error(
          "GaussianHybridFactorGraph::sum can only "
          "handleDCGaussianMixtureFactors.");
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    sum += gaussianFactor;
  }
  return sum;
}

ostream& operator<<(ostream& os,
                    const GaussianFactorGraph::EliminationResult& er) {
  os << "ER" << endl;
  return os;
}

}  // namespace gtsam
