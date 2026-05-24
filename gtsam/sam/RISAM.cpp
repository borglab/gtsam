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
    const NonlinearFactorGraph& new_factors, const Values& new_theta,
    const std::optional<std::set<Key>> extra_gnc_involved_keys,
    const FactorIndices& remove_factor_indices,
    const std::optional<FastMap<Key, int>>& constrained_keys,
    const std::optional<FastList<Key>>& no_relin_keys,
    const std::optional<FastList<Key>>& extra_reelim_keys,
    bool force_relinearize) {
  ISAM2UpdateParams update_params;
  update_params.removeFactorIndices = remove_factor_indices;
  update_params.constrainedKeys = constrained_keys;
  update_params.noRelinKeys = no_relin_keys;
  update_params.extraReelimKeys = extra_reelim_keys;
  update_params.force_relinearize = force_relinearize;
  return update(new_factors, new_theta, extra_gnc_involved_keys, update_params);
}

/* ************************************************************************* */
RISAM::UpdateResult RISAM::update(
    const NonlinearFactorGraph& new_factors, const Values& new_theta,
    const std::optional<std::set<Key>> extra_gnc_involved_keys,
    const ISAM2UpdateParams& update_params) {
  // Determine if the update includes any graduated factors
  bool update_includes_potential_outliers = false;
  for (auto& factor : new_factors) {
    auto grad_factor = std::dynamic_pointer_cast<GraduatedFactor>(factor);
    if (grad_factor) update_includes_potential_outliers = true;
  }

  // Update housekeeping for any mu_inits
  updateHouseKeeping(new_factors, update_params);

  // Run the update: robust if we have potential outliers (or if requested)
  // otherwise standard iSAM2 to improve efficiency
  UpdateResult result;
  if (extra_gnc_involved_keys || update_includes_potential_outliers) {
    result = updateRobust(new_factors, new_theta, extra_gnc_involved_keys,
                          update_params);
  } else {
    result.isam2_result =
        solver_->update(new_factors, new_theta, update_params);
    solver_->calculateEstimate();
  }

  // If we have converged update mu_inits_based on status
  if (params_.increment_outlier_mu) incrementMuInits();

  return result;
}

/* ************************************************************************* */
RISAM::UpdateResult RISAM::updateRobust(
    const NonlinearFactorGraph& new_factors, const Values& new_theta,
    const std::optional<std::set<Key>> extra_gnc_involved_keys,
    const ISAM2UpdateParams& update_params) {
  // Setup the result structure
  UpdateResult result;

  // Convexify involved factors
  ISAM2UpdateParams init_params = update_params;
  FactorIndices convex_factors = convexifyInvolvedFactors(
      new_factors, new_theta, extra_gnc_involved_keys, init_params, result);

  // Run the initial update to add new factors, after this the iSAM2
  // factors/var_index will match the risam copies
  result.isam2_result = solver_->update(new_factors, new_theta, init_params);
  solver_->calculateEstimate();

  // Update count used for some convexity graduation schedules
  std::map<FactorIndex, size_t> mu_update_count;
  for (auto& fidx : convex_factors) mu_update_count[fidx] = 0;
  /// Update until all convex factors have converged
  while (convex_factors.size() > 0) {
    convex_factors = runRobustIteration(convex_factors, mu_update_count);
  }

  // Orig RISAM preformed 1 extra iteration for better convergence
  for (size_t i = 0; i < params_.number_extra_iters; i++) {
    solver_->update(NonlinearFactorGraph(), Values(), ISAM2UpdateParams());
    solver_->calculateEstimate();
  }

  return result;
}

/* ************************************************************************* */
FactorIndices RISAM::runRobustIteration(
    const FactorIndices& convex_factors,
    std::map<FactorIndex, size_t>& mu_update_count) {
  // Get the current solution
  Values current_est = solver_->calculateEstimate();

  // Update mu for all convex factors and get the convex keys
  FastList<Key> convex_keys;
  for (FactorIndex fidx : convex_factors) {
    updateConvexFactorMu(current_est, fidx, mu_update_count, convex_keys);
  }

  // Run the Update, re-eliminating the subproblem defined at this time-step
  ISAM2UpdateParams params_internal;
  params_internal.constrainedKeys = FastMap<Key, int>();
  params_internal.extraReelimKeys = convex_keys;
  solver_->update({}, {}, params_internal);
  solver_->calculateEstimate();

  // Update set of convex Factors
  return updateConvexFactors(convex_factors);
}

/* ************************************************************************* */
void RISAM::updateConvexFactorMu(const Values& current_est, const size_t fidx,
                                 std::map<FactorIndex, size_t>& mu_update_count,
                                 FastList<Key>& convex_keys) {
  // Invariant: fidx referrs to a GraduatedFactor
  auto grad_factor =
      std::dynamic_pointer_cast<GraduatedFactor>(factors_.at(fidx));
  const double residual = grad_factor->residual(current_est);
  // Update the mu value using the factor's scheduler
  *(mu_[fidx]) = grad_factor->scheduler()->updateMu(*(mu_[fidx]), residual,
                                                    mu_update_count[fidx]);
  // Aggregate all convex keys
  convex_keys.insert(convex_keys.end(), factors_.at(fidx)->begin(),
                     factors_.at(fidx)->end());
  mu_update_count[fidx]++;
}

/* ************************************************************************* */
FactorIndices RISAM::updateConvexFactors(const FactorIndices& convex_factors) {
  FactorIndices new_convex_factors;
  // For all previously convex factors check \mu convergence
  for (FactorIndex fidx : convex_factors) {
    auto grad_factor =
        std::dynamic_pointer_cast<GraduatedFactor>(factors_.at(fidx));
    if (!grad_factor->scheduler()->isMuConverged(*(mu_[fidx]))) {
      new_convex_factors.push_back(fidx);
    }
  }
  return new_convex_factors;
}

/* ************************************************************************* */
Values RISAM::calculateEstimate() { return solver_->calculateEstimate(); }

/* ************************************************************************* */
std::set<size_t> RISAM::getOutliers(double chi2_outlier_thresh) {
  std::set<size_t> outlier_factors;
  Values current_est = solver_->calculateEstimate();

  for (size_t i = 0; i < factors_.size(); i++) {
    NonlinearFactor::shared_ptr nlf_ptr = factors_.at(i);
    GraduatedFactor::shared_ptr grad_ptr =
        std::dynamic_pointer_cast<GraduatedFactor>(nlf_ptr);
    if (grad_ptr) {
      const double thresh =
          internal::chiSquaredQuantile(nlf_ptr->dim(), chi2_outlier_thresh);
      const double residual = grad_ptr->residual(current_est);
      if (residual > thresh) outlier_factors.insert(i);
    }
  }
  return outlier_factors;
}

/* ************************************************************************* */
void RISAM::updateHouseKeeping(const NonlinearFactorGraph& new_factors,
                               const ISAM2UpdateParams& update_params) {
  // To match the behavior of iSAM2 and ensure matching indices we first add
  // info for new factors then remove info

  // Add the factors and get their new indices
  FactorIndices new_factor_indices = factors_.add_factors(
      new_factors, params_.isam2_params.findUnusedFactorSlots);
  // Update the variable index for the new factors
  variable_index_.augment(new_factors, new_factor_indices);
  // Add a mu and mu_init entry for each factor
  augmentMu(new_factors, new_factor_indices);

  // Once we add new factors we can remove any factors and corresponding info
  NonlinearFactorGraph removed_factors;
  removed_factors.reserve(update_params.removeFactorIndices.size());
  for (const auto fidx : update_params.removeFactorIndices) {
    removed_factors.push_back(factors_.at(fidx));
    factors_.remove(fidx);
    factors_to_check_status_.erase(fidx);
    mu_.at(fidx).reset();
    mu_inits_.at(fidx).reset();
  }
  variable_index_.remove(update_params.removeFactorIndices.begin(),
                         update_params.removeFactorIndices.end(),
                         removed_factors);
}

/* ************************************************************************* */
void RISAM::augmentMu(const NonlinearFactorGraph& new_factors,
                      const FactorIndices& new_factor_indices) {
  for (size_t i = 0; i < new_factors.nrFactors(); i++) {
    FactorIndex fidx = new_factor_indices[i];
    auto grad_factor =
        std::dynamic_pointer_cast<GraduatedFactor>(new_factors.at(i));

    // Get the pointers to mu and mu_init for this factor
    std::shared_ptr<double> mu, mu_init;
    if (grad_factor) {
      mu = grad_factor->mu_;
      mu_init = std::make_shared<double>(grad_factor->scheduler()->muInit());
    } else {
      mu = std::make_shared<double>(0);
      mu_init = std::make_shared<double>(0);
    }

    // Extend or replace mu and mu_inits
    if (fidx >= mu_.size()) {
      mu_.push_back(mu);
      mu_inits_.push_back(mu_init);
    } else {
      mu_[fidx] = mu;
      mu_inits_[fidx] = mu_init;
    }
  }
}

/* ************************************************************************* */
void RISAM::incrementMuInits() {
  // Compute Average Delta
  VectorValues delta = solver_->getDelta();
  bool is_sufficient_delta =
      (delta.size() > 0) && (delta.norm() / delta.size() <
                             params_.outlier_mu_avg_var_convergence_thresh);

  // Evaluate All Graduated factors if sufficient average delta
  if (is_sufficient_delta) {
    Values theta = solver_->calculateEstimate();
    for (auto fidx : factors_to_check_status_) {
      auto grad_factor =
          std::dynamic_pointer_cast<GraduatedFactor>(factors_[fidx]);

      double mahdist = grad_factor->residual(theta);
      const double mah_upper_bound = internal::chiSquaredQuantile(
          factors_[fidx]->dim(), params_.outlier_mu_chisq_upper_bound);
      const double mah_lower_bound = internal::chiSquaredQuantile(
          factors_[fidx]->dim(), params_.outlier_mu_chisq_lower_bound);

      if (mahdist > mah_upper_bound) {
        *(mu_inits_[fidx]) =
            grad_factor->scheduler()->updateMuInit(*(mu_inits_[fidx]), false);
      } else if (mahdist < mah_lower_bound) {
        *(mu_inits_[fidx]) =
            grad_factor->scheduler()->updateMuInit(*(mu_inits_[fidx]), true);
      }
    }
    // Reset accumulator
    factors_to_check_status_.clear();
  }
}

/* ************************************************************************* */
FactorIndices RISAM::convexifyInvolvedFactors(
    const NonlinearFactorGraph& new_factors, const Values& new_theta,
    const std::optional<std::set<Key>> extra_gnc_involved_keys,
    ISAM2UpdateParams& update_params, UpdateResult& update_result) {
  // Gather all involved keys - Directly induced by the new factors
  KeySet new_factor_keys = new_factors.keys();
  KeySet involved_keys = solver_->collectAffectedKeys(new_factors.keyVector());
  involved_keys.insert(new_factor_keys.begin(), new_factor_keys.end());

  // Add to the gathered involved keys, any keys specified by the user
  if (extra_gnc_involved_keys) {
    involved_keys.insert(extra_gnc_involved_keys->begin(),
                         extra_gnc_involved_keys->end());
    // For any user specified involved keys also add them to the extra-reelim
    if (!update_params.extraReelimKeys) {
      // Create container if it does not exist
      update_params.extraReelimKeys = FastList<Key>();
    }
    update_params.extraReelimKeys->insert(update_params.extraReelimKeys->end(),
                                          extra_gnc_involved_keys->begin(),
                                          extra_gnc_involved_keys->end());
  }

  // Gather all affected keys - Super set of involved keys including keys
  // modified by user params
  auto [affected_keys, is_batch_update] =
      solver_->predictUpdateInfo(new_factors, new_theta, update_params);

  // Convexify update-involved factors
  std::set<FactorIndex> convex_factors;
  for (Key affected_key : affected_keys) {
    for (FactorIndex fidx : variable_index_[affected_key]) {
      auto grad_factor =
          std::dynamic_pointer_cast<GraduatedFactor>(factors_.at(fidx));
      if (grad_factor) {
        // Indicates that all variables in this factor are in the affected set
        bool inside = true;
        // Indicates a factor touches at least one variable in the involved set
        bool update_involved = false;
        // Compute inside and update involved
        for (Key factor_key : factors_.at(fidx)->keys()) {
          inside = inside && affected_keys.count(factor_key);
          update_involved = update_involved || involved_keys.count(factor_key);
        }

        // If the factor is update involved and marked as inside, we need to
        // convexify Note: everything is inside on a batch update
        if ((inside || is_batch_update) && update_involved) {
          convex_factors.insert(fidx);
          factors_to_check_status_.insert(fidx);
          *(mu_[fidx]) = *(mu_inits_[fidx]);
        }
      }
    }
  }

  // Fill out the Update Result
  update_result.involved_variables = involved_keys;
  update_result.convexified_factors = convex_factors;
  update_result.affected_variables = affected_keys;

  // Return
  return FactorIndices(convex_factors.begin(), convex_factors.end());
}

}  // namespace gtsam