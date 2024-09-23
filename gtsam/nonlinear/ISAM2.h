/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/ISAM2Clique.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace gtsam {

/**
 * @ingroup isam2
 * Implementation of the full ISAM2 algorithm for incremental nonlinear
 * optimization.
 *
 * The typical cycle of using this class to create an instance by providing
 * ISAM2Params to the constructor, then add measurements and variables as they
 * arrive using the update() method.  At any time, calculateEstimate() may be
 * called to obtain the current estimate of all variables.
 *
 */
class GTSAM_EXPORT ISAM2 : public BayesTree<ISAM2Clique> {
 protected:
  /** The current linearization point */
  Values theta_;

  /** VariableIndex lets us look up factors by involved variable and keeps track
   * of dimensions */
  VariableIndex variableIndex_;

  /** The linear delta from the last linear solution, an update to the estimate
   * in theta
   *
   * This is \c mutable because it is a "cached" variable - it is not updated
   * until either requested with getDelta() or calculateEstimate(), or needed
   * during update() to evaluate whether to relinearize variables.
   */
  mutable VectorValues delta_;

  mutable VectorValues deltaNewton_;  // Only used when using Dogleg - stores
                                      // the Gauss-Newton update
  mutable VectorValues RgProd_;  // Only used when using Dogleg - stores R*g and
                                 // is updated incrementally

  /** A cumulative mask for the variables that were replaced and have not yet
   * been updated in the linear solution delta_, this is only used internally,
   * delta will always be updated if necessary when requested with getDelta()
   * or calculateEstimate().
   *
   * This is \c mutable because it is used internally to not update delta_
   * until it is needed.
   */
  mutable KeySet deltaReplacedMask_;  // TODO(dellaert): Make sure accessed in
                                      // the right way

  /** All original nonlinear factors are stored here to use during
   * relinearization */
  NonlinearFactorGraph nonlinearFactors_;

  /** The current linear factors, which are only updated as needed */
  mutable GaussianFactorGraph linearFactors_;

  /** The current parameters */
  ISAM2Params params_;

  /** The current Dogleg Delta (trust region radius) */
  mutable std::optional<double> doglegDelta_;

  /** Set of variables that are involved with linear factors from marginalized
   * variables and thus cannot have their linearization points changed. */
  KeySet fixedVariables_;

  int update_count_;  ///< Counter incremented every update(), used to determine
                      ///< periodic relinearization

 public:
  using This = ISAM2;                       ///< This class
  using Base = BayesTree<ISAM2Clique>;      ///< The BayesTree base class
  using Clique = Base::Clique;              ///< A clique
  using sharedClique = Base::sharedClique;  ///< Shared pointer to a clique
  using Cliques = Base::Cliques;            ///< List of Cliques

  /** Create an empty ISAM2 instance */
  explicit ISAM2(const ISAM2Params& params);

  /** Create an empty ISAM2 instance using the default set of parameters (see
   * ISAM2Params) */
  ISAM2();

  /** default virtual destructor */
  virtual ~ISAM2() {}

  /** Compare equality */
  virtual bool equals(const ISAM2& other, double tol = 1e-9) const;

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Optionally, this function remove existing factors from the system to enable
   * behaviors such as swapping existing factors with new ones.
   *
   * Add new measurements, and optionally new variables, to the current system.
   * This runs a full step of the ISAM2 algorithm, relinearizing and updating
   * the solution as needed, according to the wildfire and relinearize
   * thresholds.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the
   * system. You must include here all new variables occuring in newFactors
   * (which were not already in the system).  There must not be any variables
   * here that do not occur in newFactors, and additionally, variables that were
   * already in the system must not be included here.
   * @param removeFactorIndices Indices of factors to remove from system
   * @param force_relinearize Relinearize any variables whose delta magnitude is
   * sufficiently large (Params::relinearizeThreshold), regardless of the
   * relinearization interval (Params::relinearizeSkip).
   * @param constrainedKeys is an optional map of keys to group labels, such
   * that a variable can be constrained to a particular grouping in the
   * BayesTree
   * @param noRelinKeys is an optional set of nonlinear keys that iSAM2 will
   * hold at a constant linearization point, regardless of the size of the
   * linear delta
   * @param extraReelimKeys is an optional set of nonlinear keys that iSAM2 will
   * re-eliminate, regardless of the size of the linear delta. This allows the
   * provided keys to be reordered.
   * @return An ISAM2Result struct containing information about the update
   */
  virtual ISAM2Result update(
      const NonlinearFactorGraph& newFactors = NonlinearFactorGraph(),
      const Values& newTheta = Values(),
      const FactorIndices& removeFactorIndices = FactorIndices(),
      const std::optional<FastMap<Key, int> >& constrainedKeys = {},
      const std::optional<FastList<Key> >& noRelinKeys = {},
      const std::optional<FastList<Key> >& extraReelimKeys = {},
      bool force_relinearize = false);

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Alternative signature of update() (see its documentation above), with all
   * additional parameters in one structure. This form makes easier to keep
   * future API/ABI compatibility if parameters change.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the
   * system. You must include here all new variables occuring in newFactors
   * (which were not already in the system).  There must not be any variables
   * here that do not occur in newFactors, and additionally, variables that were
   * already in the system must not be included here.
   * @param updateParams Additional parameters to control relinearization,
   * constrained keys, etc.
   * @return An ISAM2Result struct containing information about the update
   * @note No default parameters to avoid ambiguous call errors.
   */
  virtual ISAM2Result update(const NonlinearFactorGraph& newFactors,
                             const Values& newTheta,
                             const ISAM2UpdateParams& updateParams);

  /** Marginalize out variables listed in leafKeys.  These keys must be leaves
   * in the BayesTree.  Throws MarginalizeNonleafException if non-leaves are
   * requested to be marginalized.  Marginalization leaves a linear
   * approximation of the marginal in the system, and the linearization points
   * of any variables involved in this linear marginal become fixed.  The set
   * fixed variables will include any key involved with the marginalized
   * variables in the original factors, and possibly additional ones due to
   * fill-in.
   *
   * If provided, 'marginalFactorsIndices' will be augmented with the factor
   * graph indices of the marginal factors added during the 'marginalizeLeaves'
   * call
   *
   * If provided, 'deletedFactorsIndices' will be augmented with the factor
   * graph indices of any factor that was removed during the 'marginalizeLeaves'
   * call
   */
  void marginalizeLeaves(
      const FastList<Key>& leafKeys,
      FactorIndices* marginalFactorsIndices = nullptr,
      FactorIndices* deletedFactorsIndices = nullptr);

  /** An overload of marginalizeLeaves that takes references
   * to vectors instead of pointers to vectors and passes
   * it to the pointer version of the function.
   */
  template <class... OptArgs>
      void marginalizeLeaves(const FastList<Key>& leafKeys,
                             OptArgs&&... optArgs) {
          // dereference the optional arguments and pass
          // it to the pointer version
          marginalizeLeaves(leafKeys, (&optArgs)...);
      }

  /// Access the current linearization point
  const Values& getLinearizationPoint() const { return theta_; }

  /// Check whether variable with given key exists in linearization point
  bool valueExists(Key key) const { return theta_.exists(key); }

  /** Compute an estimate from the incomplete linear delta computed during the
   * last update. This delta is incomplete because it was not updated below
   * wildfire_threshold.  If only a single variable is needed, it is faster to
   * call calculateEstimate(const KEY&).
   */
  Values calculateEstimate() const;

  /** Compute an estimate for a single variable using its incomplete linear
   * delta computed during the last update.  This is faster than calling the
   * no-argument version of calculateEstimate, which operates on all variables.
   * @param key
   * @return
   */
  template <class VALUE>
  VALUE calculateEstimate(Key key) const {
    const Vector& delta = getDelta()[key];
    return traits<VALUE>::Retract(theta_.at<VALUE>(key), delta);
  }

  /** Compute an estimate for a single variable using its incomplete linear
   * delta computed during the last update.  This is faster than calling the
   * no-argument version of calculateEstimate, which operates on all variables.
   * This is a non-templated version that returns a Value base class for use
   * with the MATLAB wrapper.
   * @param key
   * @return
   */
  const Value& calculateEstimate(Key key) const;

  /** Return marginal on any variable as a covariance matrix */
  Matrix marginalCovariance(Key key) const;

  /// @name Public members for non-typical usage
  /// @{

  /** Compute an estimate using a complete delta computed by a full
   * back-substitution.
   */
  Values calculateBestEstimate() const;

  /** Access the current delta, computed during the last call to update */
  const VectorValues& getDelta() const;

  /** Compute the linear error */
  double error(const VectorValues& x) const;

  /** Access the set of nonlinear factors */
  const NonlinearFactorGraph& getFactorsUnsafe() const {
    return nonlinearFactors_;
  }

  /** Access the nonlinear variable index */
  const VariableIndex& getVariableIndex() const { return variableIndex_; }

  /** Access the nonlinear variable index */
  const KeySet& getFixedVariables() const { return fixedVariables_; }

  const ISAM2Params& params() const { return params_; }

  /** prints out clique statistics */
  void printStats() const { getCliqueData().getStats().print(); }

  /** Compute the gradient of the energy function, \f$ \nabla_{x=0} \left\Vert
   * \Sigma^{-1} R x - d \right\Vert^2 \f$, centered around zero. The gradient
   * about zero is \f$ -R^T d \f$.  See also gradient(const GaussianBayesNet&,
   * const VectorValues&).
   *
   * @return A VectorValues storing the gradient.
   */
  VectorValues gradientAtZero() const;

  /// @}

 protected:
  /// Remove marked top and either recalculate in batch or incrementally.
  void recalculate(const ISAM2UpdateParams& updateParams,
                   const KeySet& relinKeys, ISAM2Result* result);

  // Do a batch step - reorder and relinearize all variables
  void recalculateBatch(const ISAM2UpdateParams& updateParams,
                        KeySet* affectedKeysSet, ISAM2Result* result);

  // retrieve all factors that ONLY contain the affected variables
  // (note that the remaining stuff is summarized in the cached factors)
  GaussianFactorGraph relinearizeAffectedFactors(
      const ISAM2UpdateParams& updateParams, const FastList<Key>& affectedKeys,
      const KeySet& relinKeys);

  /**
   * @brief Perform an incremental update of the factor graph to return a new
   * Bayes Tree with affected keys.
   *
   * @param updateParams Parameters for the ISAM2 update.
   * @param relinKeys Keys of variables to relinearize.
   * @param affectedKeys The set of keys which are affected in the update.
   * @param affectedKeysSet [output] Affected and contaminated keys.
   * @param orphans [output] List of orphanes cliques after elimination.
   * @param result [output] The result of the incremental update step.
   */
  void recalculateIncremental(const ISAM2UpdateParams& updateParams,
                              const KeySet& relinKeys,
                              const FastList<Key>& affectedKeys,
                              KeySet* affectedKeysSet, Cliques* orphans,
                              ISAM2Result* result);

  /**
   * Add new variables to the ISAM2 system.
   * @param newTheta Initial values for new variables
   * @param variableStatus optional detailed result structure
   */
  void addVariables(const Values& newTheta,
                    ISAM2Result::DetailedResults* detail = 0);

  /**
   * Remove variables from the ISAM2 system.
   */
  void removeVariables(const KeySet& unusedKeys);

  void updateDelta(bool forceFullSolve = false) const;

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(theta_);
      ar & BOOST_SERIALIZATION_NVP(variableIndex_);
      ar & BOOST_SERIALIZATION_NVP(delta_);
      ar & BOOST_SERIALIZATION_NVP(deltaNewton_);
      ar & BOOST_SERIALIZATION_NVP(RgProd_);
      ar & BOOST_SERIALIZATION_NVP(deltaReplacedMask_);
      ar & BOOST_SERIALIZATION_NVP(nonlinearFactors_);
      ar & BOOST_SERIALIZATION_NVP(linearFactors_);
      ar & BOOST_SERIALIZATION_NVP(doglegDelta_);
      ar & BOOST_SERIALIZATION_NVP(fixedVariables_);
      ar & BOOST_SERIALIZATION_NVP(update_count_);
  }
#endif

};  // ISAM2

/// traits
template <>
struct traits<ISAM2> : public Testable<ISAM2> {};

}  // namespace gtsam
