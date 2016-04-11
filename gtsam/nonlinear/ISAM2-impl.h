/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2-impl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess, Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {

struct GTSAM_EXPORT ISAM2::Impl {

  struct GTSAM_EXPORT PartialSolveResult {
    ISAM2::sharedClique bayesTree;
  };

  struct GTSAM_EXPORT ReorderingMode {
    size_t nFullSystemVars;
    enum { /*AS_ADDED,*/ COLAMD } algorithm;
    enum { NO_CONSTRAINT, CONSTRAIN_LAST } constrain;
    boost::optional<FastMap<Key,int> > constrainedKeys;
  };

  /**
   * Add new variables to the ISAM2 system.
   * @param newTheta Initial values for new variables
   * @param theta Current solution to be augmented with new initialization
   * @param delta Current linear delta to be augmented with zeros
   * @param ordering Current ordering to be augmented with new variables
   * @param nodes Current BayesTree::Nodes index to be augmented with slots for new variables
   * @param keyFormatter Formatter for printing nonlinear keys during debugging
   */
  static void AddVariables(const Values& newTheta, Values& theta, VectorValues& delta,
      VectorValues& deltaNewton, VectorValues& RgProd,
      const KeyFormatter& keyFormatter = DefaultKeyFormatter);

  /// Perform the first part of the bookkeeping updates for adding new factors.  Adds them to the
  /// complete list of nonlinear factors, and populates the list of new factor indices, both
  /// optionally finding and reusing empty factor slots.
  static void AddFactorsStep1(const NonlinearFactorGraph& newFactors, bool useUnusedSlots,
    NonlinearFactorGraph& nonlinearFactors, FactorIndices& newFactorIndices);
    
  /**
   * Remove variables from the ISAM2 system.
   */
  static void RemoveVariables(const KeySet& unusedKeys, const FastVector<ISAM2::sharedClique>& roots,
    Values& theta, VariableIndex& variableIndex, VectorValues& delta, VectorValues& deltaNewton,
    VectorValues& RgProd, KeySet& replacedKeys, Base::Nodes& nodes,
    KeySet& fixedVariables);

  /**
   * Find the set of variables to be relinearized according to relinearizeThreshold.
   * Any variables in the VectorValues delta whose vector magnitude is greater than
   * or equal to relinearizeThreshold are returned.
   * @param delta The linear delta to check against the threshold
   * @param keyFormatter Formatter for printing nonlinear keys during debugging
   * @return The set of variable indices in delta whose magnitude is greater than or
   * equal to relinearizeThreshold
   */
  static KeySet CheckRelinearizationFull(const VectorValues& delta,
      const ISAM2Params::RelinearizationThreshold& relinearizeThreshold);

  /**
   * Find the set of variables to be relinearized according to relinearizeThreshold.
   * This check is performed recursively, starting at the top of the tree. Once a
   * variable in the tree does not need to be relinearized, no further checks in
   * that branch are performed. This is an approximation of the Full version, designed
   * to save time at the expense of accuracy.
   * @param delta The linear delta to check against the threshold
   * @param keyFormatter Formatter for printing nonlinear keys during debugging
   * @return The set of variable indices in delta whose magnitude is greater than or
   * equal to relinearizeThreshold
   */
  static KeySet CheckRelinearizationPartial(const FastVector<ISAM2::sharedClique>& roots,
    const VectorValues& delta, const ISAM2Params::RelinearizationThreshold& relinearizeThreshold);

  /**
   * Recursively search this clique and its children for marked keys appearing
   * in the separator, and add the *frontal* keys of any cliques whose
   * separator contains any marked keys to the set \c keys.  The purpose of
   * this is to discover the cliques that need to be redone due to information
   * propagating to them from cliques that directly contain factors being
   * relinearized.
   *
   * The original comment says this finds all variables directly connected to
   * the marked ones by measurements.  Is this true, because it seems like this
   * would also pull in variables indirectly connected through other frontal or
   * separator variables?
   *
   * Alternatively could we trace up towards the root for each variable here?
   */
  static void FindAll(ISAM2Clique::shared_ptr clique, KeySet& keys, const KeySet& markedMask);

  /**
   * Apply expmap to the given values, but only for indices appearing in
   * \c markedRelinMask.  Values are expmapped in-place.
   * \param [in, out] values The value to expmap in-place
   * \param delta The linear delta with which to expmap
   * \param ordering The ordering
   * \param mask Mask on linear indices, only \c true entries are expmapped
   * \param invalidateIfDebug If this is true, *and* NDEBUG is not defined,
   * expmapped deltas will be set to an invalid value (infinity) to catch bugs
   * where we might expmap something twice, or expmap it but then not
   * recalculate its delta.
   * @param keyFormatter Formatter for printing nonlinear keys during debugging
   */
  static void ExpmapMasked(Values& values, const VectorValues& delta,
      const KeySet& mask,
      boost::optional<VectorValues&> invalidateIfDebug = boost::none,
      const KeyFormatter& keyFormatter = DefaultKeyFormatter);

  /**
   * Update the Newton's method step point, using wildfire
   */
  static size_t UpdateGaussNewtonDelta(const FastVector<ISAM2::sharedClique>& roots,
      const KeySet& replacedKeys, VectorValues& delta, double wildfireThreshold);

  /**
   * Update the RgProd (R*g) incrementally taking into account which variables
   * have been recalculated in \c replacedKeys.  Only used in Dogleg.
   */
  static size_t UpdateRgProd(const ISAM2::Roots& roots, const KeySet& replacedKeys,
      const VectorValues& gradAtZero, VectorValues& RgProd);
  
  /**
   * Compute the gradient-search point.  Only used in Dogleg.
   */
  static VectorValues ComputeGradientSearch(const VectorValues& gradAtZero,
                                            const VectorValues& RgProd);

};

}
