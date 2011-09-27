/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <map>
#include <list>
#include <vector>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>
#include <stdexcept>

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastList.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>

namespace gtsam {

/**
 * @defgroup ISAM2
 */

/**
 * @ingroup ISAM2
 * Parameters for the ISAM2 algorithm.  Default parameter values are listed below.
 */
struct ISAM2Params {
  double wildfireThreshold; ///< Continue updating the linear delta only when changes are above this threshold (default: 0.001)
  double relinearizeThreshold; ///< Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1)
  int relinearizeSkip; ///< Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)
  bool enableRelinearization; ///< Controls whether ISAM2 will ever relinearize any variables (default: true)

  /** Specify parameters as constructor arguments */
  ISAM2Params(
      double _wildfireThreshold = 0.001, ///< ISAM2Params::wildfireThreshold
      double _relinearizeThreshold = 0.1, ///< ISAM2Params::relinearizeThreshold
      int _relinearizeSkip = 10, ///< ISAM2Params::relinearizeSkip
      bool _enableRelinearization = true ///< ISAM2Params::enableRelinearization
  ) : wildfireThreshold(_wildfireThreshold), relinearizeThreshold(_relinearizeThreshold),
      relinearizeSkip(_relinearizeSkip), enableRelinearization(_enableRelinearization) {}
};

/**
 * @ingroup ISAM2
 * Implementation of the full ISAM2 algorithm for incremental nonlinear optimization.
 *
 * The typical cycle of using this class to create an instance by providing ISAM2Params
 * to the constructor, then add measurements and variables as they arrive using the update()
 * method.  At any time, calculateEstimate() may be called to obtain the current
 * estimate of all variables.
 */
template<class CONDITIONAL, class VALUES>
class ISAM2: public BayesTree<CONDITIONAL> {

protected:

  /** The current linearization point */
  VALUES theta_;

  /** VariableIndex lets us look up factors by involved variable and keeps track of dimensions */
  VariableIndex variableIndex_;

  /** The linear delta from the last linear solution, an update to the estimate in theta */
  VectorValues deltaUnpermuted_;

  /** @brief The permutation through which the deltaUnpermuted_ is
   * referenced.
   *
   * Permuting Vector entries would be slow, so for performance we
   * instead maintain this permutation through which we access the linear delta
   * indirectly
   */
  Permuted<VectorValues> delta_;

  /** All original nonlinear factors are stored here to use during relinearization */
  NonlinearFactorGraph<VALUES> nonlinearFactors_;

  /** @brief The current elimination ordering Symbols to Index (integer) keys.
   *
   * We keep it up to date as we add and reorder variables.
   */
  Ordering ordering_;

  /** The current parameters */
  ISAM2Params params_;

private:
#ifndef NDEBUG
  std::vector<bool> lastRelinVariables_;
#endif

  typedef HessianFactor CacheFactor;

public:

  typedef BayesTree<CONDITIONAL> Base; ///< The BayesTree base class
  typedef ISAM2<CONDITIONAL, VALUES> This; ///< This class

  /** Create an empty ISAM2 instance */
  ISAM2(const ISAM2Params& params);

  /** Create an empty ISAM2 instance using the default set of parameters (see ISAM2Params) */
  ISAM2();

  typedef typename BayesTree<CONDITIONAL>::sharedClique sharedClique; ///< Shared pointer to a clique
  typedef typename BayesTree<CONDITIONAL>::Cliques Cliques; ///< List of Clique typedef from base class

  /**
   * Add new factors, updating the solution and relinearizing as needed.
   *
   * Add new measurements, and optionally new variables, to the current system.
   * This runs a full step of the ISAM2 algorithm, relinearizing and updating
   * the solution as needed, according to the wildfire and relinearize
   * thresholds.
   *
   * @param newFactors The new factors to be added to the system
   * @param newTheta Initialization points for new variables to be added to the system.
   * You must include here all new variables occuring in newFactors (which were not already
   * in the system).  There must not be any variables here that do not occur in newFactors,
   * and additionally, variables that were already in the system must not be included here.
   * @param force_relinearize Relinearize any variables whose delta magnitude is sufficiently
   * large (Params::relinearizeThreshold), regardless of the relinearization interval
   * (Params::relinearizeSkip).
   */
  void update(const NonlinearFactorGraph<VALUES>& newFactors, const VALUES& newTheta,
      bool force_relinearize = false);

  /** Access the current linearization point */
  const VALUES& getLinearizationPoint() const {return theta_;}

  /** Compute an estimate from the incomplete linear delta computed during the last update.
   * This delta is incomplete because it was not updated below wildfire_threshold.
   */
  VALUES calculateEstimate() const;

  /// @name Public members for non-typical usage
  //@{

  /** Internal implementation functions */
  struct Impl;

  /** Compute an estimate using a complete delta computed by a full back-substitution.
   */
  VALUES calculateBestEstimate() const;

  /** Access the current delta, computed during the last call to update */
  const Permuted<VectorValues>& getDelta() const { return delta_; }

  /** Access the set of nonlinear factors */
  const NonlinearFactorGraph<VALUES>& getFactorsUnsafe() const { return nonlinearFactors_; }

  /** Access the current ordering */
  const Ordering& getOrdering() const { return ordering_; }

  size_t lastAffectedVariableCount;
  size_t lastAffectedFactorCount;
  size_t lastAffectedCliqueCount;
  size_t lastAffectedMarkedCount;
  size_t lastBacksubVariableCount;
  size_t lastNnzTop;

  //@}

private:

  FastList<size_t> getAffectedFactors(const FastList<Index>& keys) const;
  FactorGraph<GaussianFactor>::shared_ptr relinearizeAffectedFactors(const FastList<Index>& affectedKeys) const;
  FactorGraph<CacheFactor> getCachedBoundaryFactors(Cliques& orphans);

  boost::shared_ptr<FastSet<Index> > recalculate(const FastSet<Index>& markedKeys, const FastSet<Index>& structuralKeys,
      const FastVector<Index>& newKeys, const FactorGraph<GaussianFactor>::shared_ptr newFactors = FactorGraph<GaussianFactor>::shared_ptr());
  //	void linear_update(const GaussianFactorGraph& newFactors);

}; // ISAM2

} /// namespace gtsam
