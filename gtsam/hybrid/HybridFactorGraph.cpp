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

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

void HybridFactorGraph::print(const string& str,
                              const gtsam::KeyFormatter& keyFormatter) const {
  string prefix = str.empty() ? str : str + ".";
  cout << prefix << "size: " << size() << endl;
  discreteGraph_.print(prefix + "DiscreteFactorGraph", keyFormatter);
  dcGraph_.print(prefix + "DCFactorGraph", keyFormatter);
}

bool HybridFactorGraph::equals(const HybridFactorGraph& other,
                               double tol) const {
  return Base::equals(other, tol) &&
         discreteGraph_.equals(other.discreteGraph_, tol) &&
         dcGraph_.equals(other.dcGraph_, tol);
}

void HybridFactorGraph::clear() {
  discreteGraph_.resize(0);
  dcGraph_.resize(0);
}

DiscreteKeys HybridFactorGraph::discreteKeys() const {
  DiscreteKeys result;
  // Discrete keys from the discrete graph.
  result = discreteGraph_.discreteKeys();
  // Discrete keys from the DC factor graph.
  auto dcKeys = dcGraph_.discreteKeys();
  for (auto&& key : dcKeys) {
    // Only insert unique keys
    if (std::find(result.begin(), result.end(), key) == result.end()) {
      result.push_back(key);
    }
  }
  return result;
}

using Sum = DCGaussianMixtureFactor::Sum;
Sum HybridFactorGraph::sum() const {
  // "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      sum += *mixtureFactor;
    } else {
      throw runtime_error(
          "HybridFactorGraph::sum can only handle DCGaussianMixtureFactors.");
    }
  }
  return sum;
}

}  // namespace gtsam
