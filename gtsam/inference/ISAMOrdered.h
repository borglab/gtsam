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

#include <gtsam/inference/BayesTreeOrdered.h>

namespace gtsam {

  /**
   * A Bayes tree with an update methods that implements the iSAM algorithm.
   * Given a set of new factors, it re-eliminates the invalidated part of the tree.
   * \nosubgrouping
   */
  template<class CONDITIONAL>
  class ISAMOrdered: public BayesTreeOrdered<CONDITIONAL> {

  private:

    typedef BayesTreeOrdered<CONDITIONAL> Base;

  public:

    /// @name Standard Constructors
    /// @{

    /** Create an empty Bayes Tree */
    ISAMOrdered();

    /** Copy constructor */
    ISAMOrdered(const Base& bayesTree) :
        Base(bayesTree) {
    }

    /// @}
    /// @name Advanced Interface Interface
    /// @{

    /**
     * update the Bayes tree with a set of new factors, typically derived from measurements
     * @param newFactors is a factor graph that contains the new factors
     * @param function an elimination routine
     */
    template<class FG>
    void update(const FG& newFactors, typename FG::Eliminate function);

    typedef typename BayesTreeOrdered<CONDITIONAL>::sharedClique sharedClique;  ///<TODO: comment
    typedef typename BayesTreeOrdered<CONDITIONAL>::Cliques Cliques;            ///<TODO: comment

    /** update_internal provides access to list of orphans for drawing purposes */
    template<class FG>
    void update_internal(const FG& newFactors, Cliques& orphans,
        typename FG::Eliminate function);

    /// @}

  };

}/// namespace gtsam

#include <gtsam/inference/ISAMOrdered-inl.h>
