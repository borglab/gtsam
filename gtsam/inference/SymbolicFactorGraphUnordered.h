/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraphUnordered.h>
#include <gtsam/inference/SymbolicFactorUnordered.h>

namespace gtsam {

  /** Symbolic Factor Graph
   *  \nosubgrouping
   */
  class SymbolicFactorGraphUnordered: public FactorGraphUnordered<SymbolicFactorUnordered> {

  public:

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicFactorGraphUnordered() {}
    
    ///** Eliminate the first \c n frontal variables, returning the resulting
    // * conditional and remaining factor graph - this is very inefficient for
    // * eliminating all variables, to do that use EliminationTree or
    // * JunctionTree.  Note that this version simply calls
    // * FactorGraph<IndexFactor>::eliminateFrontals with EliminateSymbolic
    // * as the eliminate function argument.
    // */
    //GTSAM_EXPORT std::pair<sharedConditional, SymbolicFactorGraph> eliminateFrontals(size_t nFrontals) const;
    //        
    ///** Factor the factor graph into a conditional and a remaining factor graph.
    // * Given the factor graph \f$ f(X) \f$, and \c variables to factorize out
    // * \f$ V \f$, this function factorizes into \f$ f(X) = f(V;Y)f(Y) \f$, where
    // * \f$ Y := X\V \f$ are the remaining variables.  If \f$ f(X) = p(X) \f$ is
    // * a probability density or likelihood, the factorization produces a
    // * conditional probability density and a marginal \f$ p(X) = p(V|Y)p(Y) \f$.
    // *
    // * For efficiency, this function treats the variables to eliminate
    // * \c variables as fully-connected, so produces a dense (fully-connected)
    // * conditional on all of the variables in \c variables, instead of a sparse
    // * BayesNet.  If the variables are not fully-connected, it is more efficient
    // * to sequentially factorize multiple times.
    // * Note that this version simply calls
    // * FactorGraph<GaussianFactor>::eliminate with EliminateSymbolic as the eliminate
    // * function argument.
    // */
    //GTSAM_EXPORT std::pair<sharedConditional, SymbolicFactorGraph> eliminate(const std::vector<Index>& variables) const;

    ///** Eliminate a single variable, by calling SymbolicFactorGraph::eliminate. */
    //GTSAM_EXPORT std::pair<sharedConditional, SymbolicFactorGraph> eliminateOne(Index variable) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// @}
    /// @name Advanced Interface
    /// @{

    /** Push back unary factor */
    GTSAM_EXPORT void push_factor(Key key);

    /** Push back binary factor */
    GTSAM_EXPORT void push_factor(Key key1, Key key2);

    /** Push back ternary factor */
    GTSAM_EXPORT void push_factor(Key key1, Key key2, Key key3);

    /** Push back 4-way factor */
    GTSAM_EXPORT void push_factor(Key key1, Key key2, Key key3, Key key4);
  };

  /** Create a combined joint factor (new style for EliminationTree). */
  GTSAM_EXPORT IndexFactor::shared_ptr CombineSymbolic(const FactorGraph<IndexFactor>& factors,
    const FastMap<Index, std::vector<Index> >& variableSlots);

  /**
   * CombineAndEliminate provides symbolic elimination.
   * Combine and eliminate can also be called separately, but for this and
   * derived classes calling them separately generally does extra work.
   */
  GTSAM_EXPORT std::pair<boost::shared_ptr<IndexConditional>, boost::shared_ptr<IndexFactor> >
  EliminateSymbolic(const FactorGraph<IndexFactor>&, size_t nrFrontals = 1);

  /// @}

} // namespace gtsam
