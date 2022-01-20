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

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/NonlinearHybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

void NonlinearHybridFactorGraph::print(
    const string& str, const gtsam::KeyFormatter& keyFormatter) const {
  Base::print(str, keyFormatter);
  nonlinearGraph_.print("NonlinearFactorGraph", keyFormatter);
}

GaussianHybridFactorGraph NonlinearHybridFactorGraph::linearize(
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

  // Construct new GaussianNonlinearHybridFactorGraph
  return GaussianHybridFactorGraph(*gaussianFactorGraph, discreteGraph_,
                                   linearized_DC_factors);
}

bool NonlinearHybridFactorGraph::equals(const NonlinearHybridFactorGraph& other,
                                        double tol) const {
  return Base::equals(other, tol) &&
         nonlinearGraph_.equals(other.nonlinearGraph_, tol);
}

void NonlinearHybridFactorGraph::clear() {
  Base::clear();
  nonlinearGraph_.resize(0);
}

}  // namespace gtsam
