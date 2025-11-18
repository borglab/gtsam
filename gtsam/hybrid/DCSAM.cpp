/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DCSAM.cpp
 * @brief   Discrete-Continuous Smoothing and Mapping for Factored Models
 * @author  Kevin Doherty
 * @author  Varun Agrawal
 * @date    March 2025
 */

#include <gtsam/hybrid/DCSAM.h>
#include <gtsam/hybrid/DiscreteBoundaryFactor.h>

namespace gtsam {

DCSAM::DCSAM() : isam_(ISAM2(ISAM2Params())) {}

DCSAM::DCSAM(const ISAM2Params &isam_params) : isam_(ISAM2(isam_params)) {}

void DCSAM::update(const HybridNonlinearFactorGraph &hnfg,
                   const HybridValues &initialGuess) {
  // First things first: combine currContinuous_ estimate with
  // the new continuous values from initialGuess to
  // produce the full continuous variable state.
  currContinuous_.insert_or_assign(initialGuess.nonlinear());

  // Also combine currDiscrete_ estimate with new discrete values from
  // initialGuess to give a full discrete variable state.
  for (const auto &kv : initialGuess.discrete()) {
    // This will update the element with key `kv.first` if one exists, or add a
    // new element with key `kv.first` if not.
    currDiscrete_[kv.first] = kv.second;
  }

  // We individually record the continuous-only and discrete-only factors
  // so we can separately record the hybrid factors and get
  // corresponding continuous/discrete factors given
  // the current best discrete/continuous estimates.
  NonlinearFactorGraph continuousFactors;
  DiscreteFactorGraph discreteFactors;

  for (auto &factor : hnfg) {
    if (auto hybrid_factor = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      if (auto nonlinear_mixture =
              std::dynamic_pointer_cast<HybridNonlinearFactor>(hybrid_factor)) {
        hfg_.push_back(nonlinear_mixture);
      }

    } else if (auto cont_factor =
                   std::dynamic_pointer_cast<NonlinearFactor>(factor)) {
      continuousFactors.push_back(cont_factor);

    } else if (auto discrete_factor =
                   std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      discreteFactors.push_back(discrete_factor);
    }
  }

  // Record the discrete factors in `dfg_`.
  updateDiscrete(discreteFactors, initialGuess.discrete());

  // Only the initialGuess needs to be provided for the continuous solver
  // (not the entire continuous state).
  updateContinuous(continuousFactors, initialGuess.nonlinear());

  currContinuous_ = isam_.calculateEstimate();

  // Discrete info will be updated when we call update/solveDiscrete again
}

void DCSAM::update() { update(HybridNonlinearFactorGraph()); }

void DCSAM::updateDiscrete(const DiscreteFactorGraph &dfg,
                           const DiscreteValues &discreteVals) {
  for (auto &factor : dfg) {
    dfg_.push_back(factor);
  }

  // Update current discrete state estimate.
  DiscreteValues mpe = solveDiscrete();
  // Update the current discrete estimate with the latest solution.
  for (auto &&kv : mpe) {
    currDiscrete_[kv.first] = kv.second;
  }
}

void DCSAM::updateContinuous(const NonlinearFactorGraph &newFactors,
                             const Values &initialGuess) {
  // Initialize continuous factors
  NonlinearFactorGraph graph(newFactors);

  // Given the best discrete values estimate,
  // we get the corresponding continuous factors.
  for (auto &factor : hfg_) {
    if (auto nonlinear_mixture =
            std::dynamic_pointer_cast<HybridNonlinearFactor>(factor)) {
      auto sharedContinuous = nonlinear_mixture->factors()(currDiscrete_).first;
      graph.push_back(sharedContinuous);
      // Record the continuous factor
      nfg_.push_back(sharedContinuous);
    }
  }

  // Mark all affected variables, which are the continuous variables
  ISAM2UpdateParams updateParams;
  FastMap<FactorIndex, KeySet> newAffectedKeys;
  for (size_t j = 0; j < nfg_.size(); j++) {
    auto continuousFactor = nfg_[j];
    for (const Key &k : continuousFactor->keys()) {
      newAffectedKeys[j].insert(k);
    }
  }
  updateParams.newAffectedKeys = std::move(newAffectedKeys);

  // Solve for continuous values
  isam_.update(graph, initialGuess, updateParams);
}

DiscreteValues DCSAM::solveDiscrete() const {
  // First copy the recorded discrete factors
  DiscreteFactorGraph dfg(dfg_);

  // Next we add the hybrid factors as DiscreteBoundary factors
  // This step ensures we use the latest currContinuous_ values
  // to compute the discrete factors
  for (auto &factor : hfg_) {
    auto nonlinear_mixture =
        std::dynamic_pointer_cast<HybridNonlinearFactor>(factor);
    if (nonlinear_mixture) {
      auto discreteFactor = std::make_shared<DiscreteBoundaryFactor>(
          nonlinear_mixture->discreteKeys(), nonlinear_mixture->factors(),
          currContinuous_);
      dfg.push_back(discreteFactor);
    }
  }

  // Finally we optimize for the MPE
  DiscreteValues discreteVals = dfg.optimize();
  return discreteVals;
}

HybridValues DCSAM::calculateEstimate() const {
  // NOTE: if we have these cached from solves, we could presumably just return
  // the cached values.
  Values continuousVals = isam_.calculateEstimate();
  DiscreteValues discreteVals = solveDiscrete();
  HybridValues values(VectorValues(), discreteVals, continuousVals);
  return values;
}

// TODO(Varun) Create HybridMarginals
//  // NOTE separate dcmarginals class?
//  DCMarginals DCSAM::getMarginals(const NonlinearFactorGraph &graph,
//                                  const Values &continuousEst,
//                                  const DiscreteFactorGraph &dfg) {
//    return DCMarginals{Marginals(graph, continuousEst),
//                       dcsam::DiscreteMarginalsOrdered(dfg)};
//  }

}  // namespace gtsam
