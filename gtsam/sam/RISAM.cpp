/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RISAM.cpp
 * @brief   Implementation of the RISAM algorithm.
 * @author  Dan McGann
 * @date    October 2025
 */

#include <gtsam/sam/RISAM.h>
#include <gtsam/sam/RISAMGraduatedFactor.h>

namespace gtsam {

/* ************************************************************************* */
RISAM::UpdateResult RISAM::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const std::optional<std::set<Key>> extraGncInvolvedKeys,
    const FactorIndices& removeFactorIndices,
    const std::optional<FastMap<Key, int>>& constrainedKeys,
    const std::optional<FastList<Key>>& noRelinKeys,
    const std::optional<FastList<Key>>& extraReelimKeys,
    bool force_relinearize) {
  ISAM2UpdateParams updateParams;
  updateParams.removeFactorIndices = removeFactorIndices;
  updateParams.constrainedKeys = constrainedKeys;
  updateParams.noRelinKeys = noRelinKeys;
  updateParams.extraReelimKeys = extraReelimKeys;
  updateParams.force_relinearize = force_relinearize;
  return update(newFactors, newTheta, extraGncInvolvedKeys, updateParams);
}

/* ************************************************************************* */
RISAM::UpdateResult RISAM::update(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const std::optional<std::set<Key>> extraGncInvolvedKeys,
    const ISAM2UpdateParams& updateParams) {
  // Determine if the update includes any graduated factors
  bool updateIncludesPotentialOutliers = false;
  for (auto& factor : newFactors) {
    auto graduatedFactor = std::dynamic_pointer_cast<GraduatedFactor>(factor);
    if (graduatedFactor) updateIncludesPotentialOutliers = true;
  }

  // Update housekeeping for any muInits
  updateHousekeeping(newFactors, updateParams);

  // Run the update: robust if we have potential outliers (or if requested)
  // otherwise standard iSAM2 to improve efficiency
  UpdateResult result;
  if (extraGncInvolvedKeys || updateIncludesPotentialOutliers) {
    result =
        updateRobust(newFactors, newTheta, extraGncInvolvedKeys, updateParams);
  } else {
    result.isam2Result = solver_->update(newFactors, newTheta, updateParams);
    solver_->calculateEstimate();  // force back-substitution
  }

  // If we have converged update muInits based on status
  if (params_.incrementOutlierMu) incrementMuInits();

  return result;
}

/* ************************************************************************* */
Values RISAM::calculateEstimate() { return solver_->calculateEstimate(); }

/* ************************************************************************* */
std::set<size_t> RISAM::getOutliers(double chiSquaredOutlierThreshold) {
  std::set<size_t> outlierFactors;
  Values currentEstimate = solver_->calculateEstimate();

  for (size_t i = 0; i < factors_.size(); i++) {
    auto nonlinearFactor = factors_.at(i);
    auto graduatedFactor =
        std::dynamic_pointer_cast<GraduatedFactor>(nonlinearFactor);
    if (graduatedFactor) {
      const double thresh = internal::chiSquaredQuantile(
          nonlinearFactor->dim(), chiSquaredOutlierThreshold);
      const double residual = graduatedFactor->residual(currentEstimate);
      if (residual * residual > thresh) outlierFactors.insert(i);
    }
  }
  return outlierFactors;
}

/* ************************************************************************* */
RISAM::UpdateResult RISAM::updateRobust(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const std::optional<std::set<Key>>& extraGncInvolvedKeys,
    const ISAM2UpdateParams& updateParams) {
  // Setup the result structure
  UpdateResult result;

  // Convexify involved factors
  ISAM2UpdateParams initParams = updateParams;
  FactorIndices convexFactors = convexifyInvolvedFactors(
      newFactors, newTheta, extraGncInvolvedKeys, initParams, result);

  // Run the initial update to add new factors, after this the iSAM2
  // factors/variableIndex will match the risam copies
  result.isam2Result = solver_->update(newFactors, newTheta, initParams);
  solver_->calculateEstimate();  // force back-substitution

  // Update count used for some convexity graduation schedules
  std::map<FactorIndex, size_t> muUpdateCount;
  for (auto& fidx : convexFactors) muUpdateCount[fidx] = 0;
  /// Update until all convex factors have converged
  while (convexFactors.size() > 0) {
    convexFactors = runRobustIteration(convexFactors, muUpdateCount);
  }

  // Orig RISAM performed 1 extra iteration for better convergence
  for (size_t i = 0; i < params_.numberExtraIterations; i++) {
    solver_->update();
    solver_->calculateEstimate();  // force back-substitution
  }

  return result;
}

/* ************************************************************************* */
FactorIndices RISAM::runRobustIteration(
    const FactorIndices& convexFactors,
    std::map<FactorIndex, size_t>& muUpdateCount) {
  // Get the current solution
  Values currentEstimate = solver_->calculateEstimate();

  // Update mu for all convex factors and get the convex keys
  FastList<Key> convexKeys;
  for (FactorIndex fidx : convexFactors) {
    updateConvexFactorMu(currentEstimate, fidx, muUpdateCount, convexKeys);
  }

  // Run the Update, re-eliminating the subproblem defined at this time-step
  ISAM2UpdateParams internalParams;
  internalParams.constrainedKeys = FastMap<Key, int>();  // force no constraints
  internalParams.extraReelimKeys = convexKeys;
  solver_->update({}, {}, internalParams);
  solver_->calculateEstimate();  // force back-substitution

  // Update set of convex Factors
  return updateConvexFactors(convexFactors);
}

/* ************************************************************************* */
void RISAM::updateConvexFactorMu(const Values& currentEstimate,
                                 const size_t fidx,
                                 std::map<FactorIndex, size_t>& muUpdateCount,
                                 FastList<Key>& convexKeys) {
  // Invariant: fidx refers to a GraduatedFactor
  auto graduatedFactor =
      std::dynamic_pointer_cast<GraduatedFactor>(factors_.at(fidx));
  // Update the mu value using the factor's scheduler
  const double residual = graduatedFactor->residual(currentEstimate);
  *(mu_[fidx]) = graduatedFactor->scheduler()->updateMu(*(mu_[fidx]), residual,
                                                        muUpdateCount[fidx]);
  // Aggregate all convex keys
  convexKeys.insert(convexKeys.end(), factors_.at(fidx)->begin(),
                    factors_.at(fidx)->end());
  muUpdateCount[fidx]++;
}

/* ************************************************************************* */
FactorIndices RISAM::updateConvexFactors(
    const FactorIndices& convexFactors) const {
  FactorIndices newConvexFactors;
  // For all previously convex factors check \mu convergence
  for (FactorIndex fidx : convexFactors) {
    auto graduatedFactor =
        std::dynamic_pointer_cast<GraduatedFactor>(factors_.at(fidx));
    if (!graduatedFactor->scheduler()->isMuConverged(*(mu_.at(fidx)))) {
      newConvexFactors.push_back(fidx);
    }
  }
  return newConvexFactors;
}

/* ************************************************************************* */
FactorIndices RISAM::convexifyInvolvedFactors(
    const NonlinearFactorGraph& newFactors, const Values& newTheta,
    const std::optional<std::set<Key>>& extraGncInvolvedKeys,
    ISAM2UpdateParams& updateParams, UpdateResult& updateResult) {
  // Gather all involved keys - Directly induced by the new factors
  KeySet involvedKeys =
      accumulateInvolvedKeys(newFactors, extraGncInvolvedKeys, updateParams);

  // Gather all affected keys - Super set of involved keys including keys
  // modified by user params
  auto [affectedKeys, isBatchUpdate] =
      solver_->predictUpdateInfo(newFactors, newTheta, updateParams);

  // Convexify update-involved factors
  std::set<FactorIndex> convexFactors;
  for (Key affectedKey : affectedKeys) {
    for (FactorIndex fidx : variableIndex_[affectedKey]) {
      convexifyFactorIfInvolved(fidx, involvedKeys, affectedKeys, isBatchUpdate,
                                convexFactors);
    }
  }

  // Fill out the Update Result
  updateResult.involvedVariables = involvedKeys;
  updateResult.convexifiedFactors = convexFactors;
  updateResult.affectedVariables = affectedKeys;

  // Return
  return FactorIndices(convexFactors.begin(), convexFactors.end());
}

/* ************************************************************************* */
KeySet RISAM::accumulateInvolvedKeys(
    const NonlinearFactorGraph& newFactors,
    const std::optional<std::set<Key>>& extraGncInvolvedKeys,
    ISAM2UpdateParams& updateParams) const {
  // Gather all involved keys - Directly induced by the new factors
  KeySet newFactorKeys = newFactors.keys();
  KeySet involvedKeys = solver_->collectAffectedKeys(newFactors.keyVector());
  involvedKeys.insert(newFactorKeys.begin(), newFactorKeys.end());

  // Add to the gathered involved keys, any keys specified by the user
  if (extraGncInvolvedKeys) {
    involvedKeys.insert(extraGncInvolvedKeys->begin(),
                        extraGncInvolvedKeys->end());
    // For any user specified involved keys also add them to the extra-reelim
    if (!updateParams.extraReelimKeys) {
      // Create container if it does not exist
      updateParams.extraReelimKeys = FastList<Key>();
    }
    updateParams.extraReelimKeys->insert(updateParams.extraReelimKeys->end(),
                                         extraGncInvolvedKeys->begin(),
                                         extraGncInvolvedKeys->end());
  }
  return involvedKeys;
}

/* ************************************************************************* */
void RISAM::convexifyFactorIfInvolved(const FactorIndex fidx,
                                      const KeySet& involvedKeys,
                                      const KeySet& affectedKeys,
                                      const bool isBatchUpdate,
                                      std::set<FactorIndex>& convexFactors) {
  auto graduatedFactor =
      std::dynamic_pointer_cast<GraduatedFactor>(factors_.at(fidx));
  if (graduatedFactor) {
    // Indicates that all variables in this factor are in the affected set
    bool inside = true;
    // Indicates a factor touches at least one variable in the involved set
    bool updateInvolved = false;
    // Compute inside and update involved
    for (Key factorKey : factors_.at(fidx)->keys()) {
      inside = inside && affectedKeys.count(factorKey);
      updateInvolved = updateInvolved || involvedKeys.count(factorKey);
    }

    // If the factor is update involved and marked as inside, we need to
    // convexify Note: everything is inside on a batch update
    if ((inside || isBatchUpdate) && updateInvolved) {
      convexFactors.insert(fidx);
      factorsToCheckStatus_.insert(fidx);
      *(mu_[fidx]) = *(muInits_[fidx]);
    }
  }
}

/* ************************************************************************* */
void RISAM::updateHousekeeping(const NonlinearFactorGraph& newFactors,
                               const ISAM2UpdateParams& updateParams) {
  // To match the behavior of iSAM2 and ensure matching indices we first add
  // info for new factors then remove info

  // Add the factors and get their new indices
  FactorIndices newFactorIndices = factors_.add_factors(
      newFactors, params_.isam2Params.findUnusedFactorSlots);
  // Update the variable index for the new factors
  variableIndex_.augment(newFactors, newFactorIndices);
  // Add a mu and muInit entry for each factor
  augmentMu(newFactors, newFactorIndices);

  // Once we add new factors we can remove any factors and corresponding info
  NonlinearFactorGraph removedFactors;
  removedFactors.reserve(updateParams.removeFactorIndices.size());
  for (const auto fidx : updateParams.removeFactorIndices) {
    removedFactors.push_back(factors_.at(fidx));
    factors_.remove(fidx);
    factorsToCheckStatus_.erase(fidx);
    mu_.at(fidx).reset();
    muInits_.at(fidx).reset();
  }
  variableIndex_.remove(updateParams.removeFactorIndices.begin(),
                        updateParams.removeFactorIndices.end(), removedFactors);
}

/* ************************************************************************* */
void RISAM::augmentMu(const NonlinearFactorGraph& newFactors,
                      const FactorIndices& newFactorIndices) {
  for (size_t i = 0; i < newFactors.size(); i++) {
    FactorIndex fidx = newFactorIndices[i];
    auto graduatedFactor =
        std::dynamic_pointer_cast<GraduatedFactor>(newFactors.at(i));

    // Get the pointers to mu and muInit for this factor
    std::shared_ptr<double> mu, muInit;
    if (graduatedFactor) {
      mu = graduatedFactor->mu_;
      muInit = std::make_shared<double>(graduatedFactor->scheduler()->muInit());
    } else {
      mu = std::make_shared<double>(0);
      muInit = std::make_shared<double>(0);
    }

    // Extend or replace mu and muInits
    if (fidx >= mu_.size()) {
      mu_.push_back(mu);
      muInits_.push_back(muInit);
    } else {
      mu_[fidx] = mu;
      muInits_[fidx] = muInit;
    }
  }
}

/* ************************************************************************* */
void RISAM::incrementMuInits() {
  // Compute Average Delta
  VectorValues delta = solver_->getDelta();
  bool isSmallDelta = (delta.size() > 0) &&
                      (delta.norm() / delta.size() <
                       params_.outlierMuAverageVariableConvergenceThreshold);

  // Evaluate All Graduated factors if small enough average delta
  if (isSmallDelta) {
    Values theta = solver_->calculateEstimate();
    for (auto fidx : factorsToCheckStatus_) {
      auto graduatedFactor =
          std::dynamic_pointer_cast<GraduatedFactor>(factors_[fidx]);

      double mahalanobisDistance = graduatedFactor->residual(theta);
      double mahalanobisDistanceSquared =
          mahalanobisDistance * mahalanobisDistance;
      const double upperBound = internal::chiSquaredQuantile(
          factors_[fidx]->dim(), params_.outlierMuChiSquaredUpperBound);
      const double lowerBound = internal::chiSquaredQuantile(
          factors_[fidx]->dim(), params_.outlierMuChiSquaredLowerBound);

      if (mahalanobisDistanceSquared > upperBound) {
        *(muInits_[fidx]) = graduatedFactor->scheduler()->updateMuInit(
            *(muInits_[fidx]), false);
      } else if (mahalanobisDistanceSquared < lowerBound) {
        *(muInits_[fidx]) =
            graduatedFactor->scheduler()->updateMuInit(*(muInits_[fidx]), true);
      }
    }
    // Reset accumulator
    factorsToCheckStatus_.clear();
  }
}

}  // namespace gtsam
