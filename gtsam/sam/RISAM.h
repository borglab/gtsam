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

/** @brief Struct Containing all configuration parameters for riSAM.
 * See below for details on each parameters.
 * @note Defined at namespace scope, and aliased as RISAM::Parameters
 */
struct RISAMParams {
  /// @brief Explicit constructor to use default values.
  RISAMParams(ISAM2Params isam2Params = ISAM2Params(),
              bool incrementOutlierMu = true,
              double outlierMuChiSquaredUpperBound = 0.95,
              double outlierMuChiSquaredLowerBound = 0.25,
              double outlierMuAverageVariableConvergenceThreshold = 0.01,
              size_t numberExtraIterations = 1)
      : isam2Params(isam2Params),
        incrementOutlierMu(incrementOutlierMu),
        outlierMuChiSquaredUpperBound(outlierMuChiSquaredUpperBound),
        outlierMuChiSquaredLowerBound(outlierMuChiSquaredLowerBound),
        outlierMuAverageVariableConvergenceThreshold(
            outlierMuAverageVariableConvergenceThreshold),
        numberExtraIterations(numberExtraIterations) {}

  /// @brief The parameters for the encapsulated iSAM2 algorithm @Note some
  /// are overridden in RISAM::RISAM().
  ISAM2Params isam2Params;

  /// @brief Flag to increment muInit when the value estimate converges.
  // See RISAM::incrementMuInits for details.
  bool incrementOutlierMu;
  /// @brief Increment muInit if chi^2 > upper bound.
  double outlierMuChiSquaredUpperBound;
  /// @brief  Decrement muInit if chi^2 < lower bound.
  double outlierMuChiSquaredLowerBound;
  /// @brief Average variable delta threshold to identify value convergence.
  double outlierMuAverageVariableConvergenceThreshold;
  /// @brief The number of extra iterations to perform after mu convergence.
  size_t numberExtraIterations;
};

/** @brief Struct containing information about the riSAM update.
 * See below for details about the information included.
 * @note Defined at namespace scope, and aliased as RISAM::UpdateResult
 */
struct RISAMUpdateResult {
  /// @brief The iSAM2 result from the first internal update.
  /// NOTE: riSAM may run multiple internal ISAM2 updates.
  ISAM2Result isam2Result;

  /// @brief The set variables directly involved in the update.
  std::set<Key> involvedVariables;
  /// @brief The set of variables affected by the update.
  std::set<Key> affectedVariables;
  /// @brief The set of factors convexified in this update.
  std::set<FactorIndex> convexifiedFactors;
};

/** @brief Robust Incremental Smoothing and Mapping (riSAM) is a robust variant
 * of iSAM2 for incremental factor-graph optimization. riSAM solves each
 * incremental update using an efficient form of Graduated Non-Convexity to
 * reject outliers while maintaining robustness to initialization.
 *
 * Citation:
 * Robust Incremental Smoothing and Mapping (riSAM)
 * D. McGann and J.G. Rogers III and M. Kaess,
 * 2023, Proc. IEEE Intl. Conf. on Robotics and Automation (ICRA)
 */
class GTSAM_EXPORT RISAM {
  /// @name Types
  /// @{
 public:
  /// Configuration parameters for the riSAM algorithm.
  typedef RISAMParams Parameters;
  /// Information about a riSAM update.
  typedef RISAMUpdateResult UpdateResult;
  /// @}

  /// @name Fields
  /// @{
 protected:
  /// @brief Configuration parameters for the riSAM algorithm.
  Parameters params_;
  /// @brief The encapsulated iSAM2 algorithm.
  std::unique_ptr<ISAM2> solver_;
  /// @brief The current control parameter values for all factors. If factor i
  /// is a GraduatedFactor mu_[i] is a reference to that factors internal state.
  /// If factor i is not Graduated mu_[i] is a pointer to a value of zero.
  FastVector<std::shared_ptr<double>> mu_;
  /// @brief The current initial control parameter values for all factors. If
  /// factor i is not Graduated muInits_[i] is a pointer to a value of zero.
  FastVector<std::shared_ptr<double>> muInits_;
  /// @brief The set of GraduatedFactors that have been convexified since the
  /// last muInit_ increment These are the factors that we can increment mu's
  /// for the next time variables converge.
  FastSet<size_t> factorsToCheckStatus_;

  /**
   * RISAM maintains its own variable index and factors. These match exactly
   * that of the underlying iSAM2 solver, but lead ahead of it since we update
   * RISAM housekeeping before updating the underlying solver.
   */
  /// @brief VariableIndex for the underlying system.
  /// INVARIANT: matches that of solver_ after every update.
  VariableIndex variableIndex_;
  /// @brief The Factors for the underlying system.
  /// INVARIANT: matches that of solver_ after every update.
  NonlinearFactorGraph factors_;
  /// @}

  /// @name Public Interface
  /// @{
 public:
  /** @brief Constructs an instance of the riSAM algorithm with provided
   * configuration. Note this constructor will override some values in
   * params.isam2Params.
   * @param params: The configuration parameters for this instance of riSAM.
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
    // We must have relinearization skip as 1.
    params_.isam2Params.relinearizeSkip = 1;
    // We must not use cached factors.
    params_.isam2Params.cacheLinearizedFactors = false;

    // Construct the encapsulated solver using the modified parameters.
    solver_ = std::make_unique<ISAM2>(params_.isam2Params);
  }

  /// @brief Update Interface. See ISAM2 docs for details as parameters match
  /// (almost) exactly.
  /// @param extraGncInvolvedKeys - overrides internal RISAM logic and
  /// performs a robust update. All extra involved keys will be treated as being
  /// a part of the current update with respect to factor convexification.
  UpdateResult update(
      const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
      const Values& newTheta = Values(),
      const std::optional<std::set<Key>> extraGncInvolvedKeys = std::nullopt,
      const FactorIndices& removeFactorIndices = FactorIndices(),
      const std::optional<FastMap<Key, int>>& constrainedKeys = std::nullopt,
      const std::optional<FastList<Key>>& noRelinKeys = std::nullopt,
      const std::optional<FastList<Key>>& extraReelimKeys = std::nullopt,
      bool force_relinearize = false);

  /// @brief Update Interface. See ISAM2 docs for details as parameters match
  /// (almost) exactly.
  /// @param extraGncInvolvedKeys - overrides internal RISAM logic and
  /// performs a robust update. All extra involved keys will be treated as being
  /// a part of the current update with respect to factor convexification.
  UpdateResult update(const NonlinearFactorGraph& newFactors,
                      const Values& newTheta,
                      const std::optional<std::set<Key>> extraGncInvolvedKeys,
                      const ISAM2UpdateParams& updateParams);

  /// @brief Returns the current estimate from the solver.
  Values calculateEstimate();

  /// @brief Returns the underlying factors of the system.
  const NonlinearFactorGraph& getFactorsUnsafe() const { return factors_; }

  /** @brief Returns the set of measurements (identified by their factor index)
   * that have been deemed outliers. riSAM defines a measurement as an outlier
   * if its current residual is greater than the chi-squared threshold provided.
   * @param chiSquaredOutlierThreshold - The chi-squared threshold used to
   * define outliers. WARN: Potentially slow since we iterate over all factors.
   * @returns The set of factors that are considered outliers by RISAM.
   */
  std::set<size_t> getOutliers(double chiSquaredOutlierThreshold);
  /// @}

  /// @name Private Interface
  /// @{
 protected:
  /** @brief Performs a robust update to the system.
   * A robust update is required any time newFactors contains GraduatedFactors
   * (i.e. there are potentially new inlier/outlier measurements).
   * @see RISAM::update for details on params.
   */
  UpdateResult updateRobust(
      const NonlinearFactorGraph& newFactors, const Values& newTheta,
      const std::optional<std::set<Key>>& extraGncInvolvedKeys,
      const ISAM2UpdateParams& updateParams);

  /** @brief Performs a inner loop iteration of a robust update.
   * @param convexFactors: The set of factors that are convex for this update.
   * @param muUpdateCount: Accum. for # of updates applied to convex factors.
   * @returns The set of factors that remain convex after this iteration.
   */
  FactorIndices runRobustIteration(
      const FactorIndices& convexFactors,
      std::map<FactorIndex, size_t>& muUpdateCount);

  /** @brief Compute the new value of $\mu$ for a given graduated factor.
   * @param currentEstimate: The current estimated solution.
   * @param fidx: The index of the graduated factor to update.
   * @param muUpdateCount: Accum. for # of updates applied to convex factors.
   * @param convexKeys: Accum. for all keys associated with convex factors.
   * @note INVARIANT: fidx must index a Graduated Factor.
   */
  void updateConvexFactorMu(const Values& currentEstimate, const size_t fidx,
                            std::map<FactorIndex, size_t>& muUpdateCount,
                            FastList<Key>& convexKeys);

  /** @brief Compute the new set of convex factors after a robust iteration.
   * @param convexFactors: The set of convex factors after the prev. iter.
   * @return The set of factors that are still convex after the current iter.
   */
  FactorIndices updateConvexFactors(const FactorIndices& convexFactors) const;

  /** @brief Convexifies factors involved in the update
   * Finds all factors inside the total affected set, and involved in the update
   * defined by new factors. Marks those factors as convex and resets mu_ to the
   * factors current muInit_. See RISAM::update for adtl. parameter details.
   * @param updateResult: structure containing information about this update,
   * modified by this function to fill in info about involved and affected
   * variables as well as convexified factors.
   * @returns The indices of all factors convexified in the update defined by
   * the given parameters.
   */
  FactorIndices convexifyInvolvedFactors(
      const NonlinearFactorGraph& newFactors, const Values& newTheta,
      const std::optional<std::set<Key>>& extraGncInvolvedKeys,
      ISAM2UpdateParams& internalUpdateParams, UpdateResult& updateResult);

  /** @brief Accumulates the keys of all vars directly involved in an update.
   * @param newFactors The new factors for the update.
   * @param extraGncInvolvedKeys: User supplied involved keys.
   * @param updateParams: The parameters for the underlying iSAM2 update.
   * @returns The set of involved factors.
   */
  KeySet accumulateInvolvedKeys(
      const NonlinearFactorGraph& newFactors,
      const std::optional<std::set<Key>>& extraGncInvolvedKeys,
      ISAM2UpdateParams& updateParams) const;

  /** @brief Convexifies a single factor if it is involved with the update.
   * @param fidx: The factor to convexify if involved.
   * @param involvedKeys: The set of keys involved with the update.
   * @param affectedKeys: The set of keys affected by the update.
   * @param isBatchUpdate: Flag indicating all factors are relinearized.
   * @param convexFactors: Accumulator for the set of all convex factors.
   */
  void convexifyFactorIfInvolved(const FactorIndex fidx,
                                 const KeySet& involvedKeys,
                                 const KeySet& affectedKeys,
                                 const bool isBatchUpdate,
                                 std::set<FactorIndex>& convexFactors);

  /** @brief Update housekeeping information for riSAM this involves:
   * 1. Determining the indices for newFactors.
   * 2. Update the riSAM fields: factors_ and variableIndex_.
   * NOTE: they will be ahead of those in solver_ until solver.update() is
   * called.
   * 3. Update mu_ and muInit_ for the new factors.
   * @param newFactors: The new factors for the current update.
   * @param updateParams: The update parameters for the current update.
   */
  void updateHousekeeping(const NonlinearFactorGraph& newFactors,
                          const ISAM2UpdateParams& updateParams);

  /** @brief Extend or update mu and muInit for new factors.
   * @param newFactors: The new factors for which to extend the containers.
   * @param newFactorIndices: the indices assigned to each new factor.
   */
  void augmentMu(const NonlinearFactorGraph& newFactors,
                 const FactorIndices& newFactorIndices);

  /** @brief Increments muInits_ for any factor recently convexified.
   * Used to mitigate the long-term effect of measurements that we are confident
   * are outliers.
   */
  void incrementMuInits();
  /// @}

  /// @name Static Helpers
  /// @{
 public:
  /** @brief Returns a factory that constructs FACTOR_TYPE graduated factors for
   * riSAM, identifying them as potential outlier measurements.
   *
   * The graduation policy and the factor's own arguments are supplied
   * separately, as MakeGraduated<FACTOR_TYPE>(graduation arguments)(factor
   * arguments), so one factory can build many factors that share a graduation
   * policy. Every constructed factor still carries its own graduation state.
   *
   * @param loss: The graduated robust loss applied to the factors
   * @param scheduler: The control param $\mu$ scheduler for the factors
   */
  template <class FACTOR_TYPE>
  static auto MakeGraduated(GraduatedFactor::RobustLoss::shared_ptr loss,
                            GraduationScheduler::shared_ptr scheduler) {
    return [loss = std::move(loss),
            scheduler = std::move(scheduler)](auto&&... factorArgs) ->
           typename GenericGraduatedFactor<FACTOR_TYPE>::shared_ptr {
             return std::make_shared<GenericGraduatedFactor<FACTOR_TYPE>>(
                 loss, scheduler,
                 std::forward<decltype(factorArgs)>(factorArgs)...);
           };
  }
  /// @}
};

}  // namespace gtsam
