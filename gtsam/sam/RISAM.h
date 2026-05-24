/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   RISAM.h
 *  @brief  Robust Incremental Smoothing and Mapping (riSAM)
 *  @author Dan McGann
 *  @date   October 2025
 */
#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/internal/ChiSquaredInverse.h>
#include <gtsam/sam/RISAMGraduatedFactor.h>

namespace gtsam {

class GTSAM_EXPORT RISAM {
  /// @name Types
  /// @{
 public:
  /** @brief Struct Containing all configuration parameters for riSAM
   * See below for details on each parameters
   */
  struct Parameters {
    /// @brief Explicit constructor to use default values
    Parameters(ISAM2Params isam2_params = ISAM2Params(),
               bool increment_outlier_mu = true,
               double outlier_mu_chisq_upper_bound = 0.95,
               double outlier_mu_chisq_lower_bound = 0.25,
               double outlier_mu_avg_var_convergence_thresh = 0.01,
               size_t number_extra_iters = 1)
        : isam2_params(isam2_params),
          increment_outlier_mu(increment_outlier_mu),
          outlier_mu_chisq_upper_bound(outlier_mu_chisq_upper_bound),
          outlier_mu_chisq_lower_bound(outlier_mu_chisq_lower_bound),
          outlier_mu_avg_var_convergence_thresh(
              outlier_mu_avg_var_convergence_thresh),
          number_extra_iters(number_extra_iters) {};
    /// @brief The parameters for the encapsulated iSAM2 algorithm @Note some
    /// are overriden in RISAM::RISAM()
    ISAM2Params isam2_params;

    /// @brief Flag to increment mu_init when the value estimate converges
    // See RISAM::IncrementMuInit for details
    bool increment_outlier_mu;
    /// @brief Increment mu_init if chi^2 > upper bound
    double outlier_mu_chisq_upper_bound;
    /// @brief  Decrement mu_init if chi^2 < lower bound
    double outlier_mu_chisq_lower_bound;
    /// @brief Average variable delta threshold to identify value convergence
    double outlier_mu_avg_var_convergence_thresh;
    /// @brief The number of extra iterations to perform after mu convergence
    size_t number_extra_iters;
  };

  /** @brief Struct containing information about the riSAM update
   * See below for details about the information included
   */
  struct UpdateResult {
    /// @brief The iSAM2 result from the first internal update
    /// NOTE: riSAM may run multiple internal ISAM2 updates
    ISAM2Result isam2_result;

    /// @brief The set variables directly involved in the update
    std::set<Key> involved_variables;
    /// @brief The set of variables affected by the update
    std::set<Key> affected_variables;
    /// @brief The set of factors convexified in this update
    std::set<FactorIndex> convexified_factors;
  };
  /// @}

  /// @name Fields
  /// @{
 protected:
  /// @brief Configuration parameters for the riSAM algorithm
  Parameters params_;
  /// @brief The encapsulated iSAM2 algorithm.
  std::unique_ptr<ISAM2> solver_;
  /// @brief The current control parameter values for all factors. If factor i
  /// is a GraduatedFactor mu_[i] is a reference to that factors internal state.
  /// If factor i is not Graduated mu_[i] is a pointer to a value of zero
  FastVector<std::shared_ptr<double>> mu_;
  /// @brief The current initial control parameter values for all factors. If
  /// factor i is not Graduated mu_[i] is a pointer to a value of zero
  FastVector<std::shared_ptr<double>> mu_inits_;
  /// @brief The set of GraduatedFactors that have been convexified since the
  /// last mu_init_ increment These are the factors that we can increment mu's
  /// for the next time variables converge.
  FastSet<size_t> factors_to_check_status_;

  /**
   * RISAM maintains its own variable index and factors. These match exactly
   * that of the underlying iSAM2 solver, but lead ahead of it since we update
   * RISAM housekeeping before updating the underlying solver.
   */
  /// @brief VariableIndex for the underlying system
  /// INVARIANT: matches that of solver_ after every update
  VariableIndex variable_index_;
  /// @brief The Factors for the underlying system
  /// INVARIANT: matches that of solver_ after every update
  NonlinearFactorGraph factors_;
  /// @}

  /// @name Public Interface
  /// @{
 public:
  /** @brief Constructs an instance of the riSAM algorithm with provided
   * configuration. Note this constructor will override some values in
   * params.isam2_params.
   * @param params: The configuration parameters for this instance of riSAM
   */
  RISAM(const Parameters& params) : params_(params) {
    /** Override user preferences for iSAM2 params
     * To ensure that we update the current estimated theta_ after each
     * intermediate GNC iteration we must use relinearizeSkip = 1 so that we
     * allow relinearization at any step it is needed.
     *
     * For each intermediate GNC iteration we update mu_ values for convex
     * GraduatedFactors to ensure that GraduatedFactors are linearized according
     * to these mu_ values we must turn off caching that could result in old mu_
     * being used.
     */
    // We must have relinearization skip as 1
    params_.isam2_params.relinearizeSkip = 1;
    // We must not use cached factors
    params_.isam2_params.cacheLinearizedFactors = false;

    // Construct the encapsulated solver using the modified parameters.
    solver_ = std::make_unique<ISAM2>(params_.isam2_params);
  }

  /// @brief Update Interface. See ISAM2 docs for details as parameters match
  /// (almost) exactly
  /// @param extra_gnc_involved_keys - overrides internal RISAM logic and
  /// performs a robust update. All extra involved keys will be treated as being
  /// a part of the current update with respect to factor convexification
  UpdateResult update(
      const NonlinearFactorGraph& new_factors = NonlinearFactorGraph(),
      const Values& new_theta = Values(),
      const std::optional<std::set<Key>> extra_gnc_involved_keys = std::nullopt,
      const FactorIndices& remove_factor_indices = FactorIndices(),
      const std::optional<FastMap<Key, int>>& constrained_keys = std::nullopt,
      const std::optional<FastList<Key>>& no_relin_keys = std::nullopt,
      const std::optional<FastList<Key>>& extra_reelim_keys = std::nullopt,
      bool force_relinearize = false);

  /// @brief Update Interface. See ISAM2 docs for details as parameters match
  /// (almost) exactly
  /// @param force_gnc_solve - overrides internal RISAM logic and performs a
  /// robust update
  UpdateResult update(
      const NonlinearFactorGraph& new_factors, const Values& new_theta,
      const std::optional<std::set<Key>> extra_gnc_involved_keys,
      const ISAM2UpdateParams& update_params);

  /// @brief Returns the current estimate from the solver
  Values calculateEstimate();

  /// @brief Returns the underlying factors of the system
  NonlinearFactorGraph getFactorsUnsafe() { return factors_; }

  /** @brief Returns the set of measurements (identified by their factor index)
   * that have been deemed outliers. riSAM defines a measurement as an outlier
   * if its current residual is greater than the chi2_threshold provided
   * @param chi2_outlier_thresh - The chi2 threshold used to define outliers
   * WARN: Potentially slow since we iterate over all factors
   * @returns The set of factors that are considered outliers by the RISAM
   * algorithm.
   */
  std::set<size_t> getOutliers(double chi2_outlier_thresh);
  /// @}

  /// @name Private Interface
  /// @{
 protected:
  /** @brief Preforms a robust update to the system.
   * A robust update is required any time new_factors contains GraduatedFactors
   * (i.e. there are potentially new inlier/outlier measurements)
   * @see ISAM2::update for details on params
   */
  UpdateResult updateRobust(
      const NonlinearFactorGraph& new_factors, const Values& new_theta,
      const std::optional<std::set<Key>> extra_gnc_involved_keys,
      const ISAM2UpdateParams& update_params);

  /**
   *
   */
  void runRobustIteration(FactorIndices& convex_factors,
                          std::map<FactorIndex, size_t>& mu_update_count,
                          ISAM2UpdateParams& update_params_internal);

  /**
   *
   */
  void updateConvexFactorMu(const Values& current_est, const size_t fidx,
                            std::map<FactorIndex, size_t>& mu_update_count,
                            FastList<Key>& convex_keys);

  void updateConvexFactors(FactorIndices& convex_factors);

  /** @brief Update housekeeping information for riSAM this involves:
   * 1. Determining the indicies for new_factors
   * 2. Update the riSAM fields: factors_ and variable_index_
   * NOTE: they will be ahead of those in solver_ until solver.update() is
   * called
   * 3. Update mu_ and mu_init_ for the new factors
   * @param new_factors: The new factors for the current update
   * @param update_params: The update parameters for the current update
   */
  void updateHouseKeeping(const NonlinearFactorGraph& new_factors,
                          const ISAM2UpdateParams& update_params);

  /** @brief Extend or update mu and mu_init for new factors
   * @param new_factors: The new factors for which to extend the containers
   * @param new_factor_indicies: the indicies assigned to each new_factor
   */
  void augmentMu(const NonlinearFactorGraph& new_factors,
                 const FactorIndices& new_factor_indices);

  /** @brief Increments mu_inits_ for any factor recently convexified.
   * Used to mitigate the long-term affect of measurements that we are confident
   * are outliers.
   */
  void incrementMuInits();

  /** @brief Convexifies factors involved in the update
   * Finds all factors inside the total affected set, and involved in the update
   * defined by new factors. Marks those factors as convex and resets mu_ to the
   * factors current mu_init_ See RISAM::update for first three parameter
   * details
   * @param update_result: structure containing information about this update,
   * modified by this function to fill in info about involved and affected
   * variables as well as convexified factors
   * @returns The indicies of all factors convexified in the update defined by
   * the given parameters.
   */
  FactorIndices convexifyInvolvedFactors(
      const NonlinearFactorGraph& new_factors, const Values& new_theta,
      const std::optional<std::set<Key>> extra_gnc_involved_keys,
      ISAM2UpdateParams& internal_update_params, UpdateResult& update_result);
  /// @}

  /// @name Static Helpers
  /// @{
 public:
  /** @brief Constructs a shared_ptr of FACTOR_TYPE graduated factor for riSAM
   * This identifies a factor as a potential outlier measurement.
   * @param kernel: The graduated kernel to use for this factor
   * @param args: The arguments required to construct the FACTOR_TYPE
   */
  template <class FACTOR_TYPE, class... Args>
  static typename GenericGraduatedFactor<FACTOR_TYPE>::shared_ptr
  make_graduated(Args&&... args) {
    return std::make_shared<GenericGraduatedFactor<FACTOR_TYPE>>(
        std::forward<Args>(args)...);
  }
  /// @}
};

}  // namespace gtsam