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
    boost::optional<FastMap<Key, int> > constrainedKeys;
  };

  /// Perform the first part of the bookkeeping updates for adding new factors.  Adds them to the
  /// complete list of nonlinear factors, and populates the list of new factor indices, both
  /// optionally finding and reusing empty factor slots.
  static void AddFactorsStep1(const NonlinearFactorGraph& newFactors, bool useUnusedSlots,
    NonlinearFactorGraph* nonlinearFactors, FactorIndices* newFactorIndices);

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
  static KeySet CheckRelinearizationPartial(const ISAM2::Roots& roots,
    const VectorValues& delta, const ISAM2Params::RelinearizationThreshold& relinearizeThreshold);

  /**
   * Update the Newton's method step point, using wildfire
   */
  static size_t UpdateGaussNewtonDelta(const ISAM2::Roots& roots,
      const KeySet& replacedKeys, double wildfireThreshold, VectorValues* delta);

  /**
   * Update the RgProd (R*g) incrementally taking into account which variables
   * have been recalculated in \c replacedKeys.  Only used in Dogleg.
   */
  static size_t UpdateRgProd(const ISAM2::Roots& roots, const KeySet& replacedKeys,
      const VectorValues& gradAtZero, VectorValues* RgProd);

  /**
   * Compute the gradient-search point.  Only used in Dogleg.
   */
  static VectorValues ComputeGradientSearch(const VectorValues& gradAtZero,
                                            const VectorValues& RgProd);

};

}
