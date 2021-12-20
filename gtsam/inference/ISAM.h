/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM.h
 * @brief   Incremental update functionality (iSAM) for BayesTree.
 * @author  Michael Kaess
 */

// \callgraph
#pragma once

#include <gtsam/global_includes.h>

namespace gtsam {

/**
 * A Bayes tree with an update methods that implements the iSAM algorithm.
 * Given a set of new factors, it re-eliminates the invalidated part of the
 * tree. \nosubgrouping
 */
template <class BAYESTREE>
class ISAM : public BAYESTREE {
 public:
  typedef BAYESTREE Base;
  typedef typename Base::BayesNetType BayesNetType;
  typedef typename Base::FactorGraphType FactorGraphType;
  typedef typename Base::Clique Clique;
  typedef typename Base::sharedClique sharedClique;
  typedef typename Base::Cliques Cliques;

 private:
  typedef typename Base::Eliminate Eliminate;
  typedef typename Base::EliminationTraitsType EliminationTraitsType;

 public:
  /// @name Standard Constructors
  /// @{

  /** Create an empty Bayes Tree */
  ISAM() {}

  /** Copy constructor */
  explicit ISAM(const Base& bayesTree) : Base(bayesTree) {}

  /// @}
  /// @name Advanced Interface Interface
  /// @{

  /**
   * update the Bayes tree with a set of new factors, typically derived from
   * measurements
   * @param newFactors is a factor graph that contains the new factors
   * @param function an elimination routine
   */
  void update(
      const FactorGraphType& newFactors,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate);

  /** updateInternal provides access to list of orphans for drawing purposes
   */
  void updateInternal(
      const FactorGraphType& newFactors, Cliques* orphans,
      const Eliminate& function = EliminationTraitsType::DefaultEliminate);

  /// @}
};

}  // namespace gtsam
