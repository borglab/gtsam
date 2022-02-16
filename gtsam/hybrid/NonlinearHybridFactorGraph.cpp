/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   NonlinearHybridFactorGraph.cpp
 * @brief  Custom hybrid factor graph for discrete + nonlinear continuous
 * factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/hybrid/NonlinearHybridFactorGraph.h>

// Needed for DecisionTree
#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

void NonlinearHybridFactorGraph::print(
    const string& str, const gtsam::KeyFormatter& keyFormatter) const {
  Base::print(str, keyFormatter);
  factorGraph_.print("NonlinearFactorGraph", keyFormatter);
}

bool NonlinearHybridFactorGraph::equals(const NonlinearHybridFactorGraph& other,
                                        double tol) const {
  return Base::equals(other, tol);
}

GaussianHybridFactorGraph NonlinearHybridFactorGraph::linearize(
    const Values& continuousValues) const {
  // linearize the continuous factors
  auto gaussianFactorGraph = factorGraph_.linearize(continuousValues);

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

  // Construct new GaussianNonlinearHybridFactorGraph
  return GaussianHybridFactorGraph(*gaussianFactorGraph, discreteGraph_,
                                   linearized_DC_factors);
}

}  // namespace gtsam
