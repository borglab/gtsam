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

#include <gtsam/global_includes.h>
#include <gtsam/inference/FactorGraphOrdered.h>
#include <gtsam/inference/IndexFactorOrdered.h>

namespace gtsam { template<class FACTOR> class EliminationTreeOrdered; }
namespace gtsam { template<class CONDITIONAL> class BayesNetOrdered; }
namespace gtsam { template<class CONDITIONAL, class CLIQUE> class BayesTreeOrdered; }
namespace gtsam { class IndexConditionalOrdered; }

namespace gtsam {

  typedef EliminationTreeOrdered<IndexFactorOrdered> SymbolicEliminationTreeOrdered;
  typedef BayesNetOrdered<IndexConditionalOrdered> SymbolicBayesNetOrdered;
  typedef BayesTreeOrdered<IndexConditionalOrdered> SymbolicBayesTreeOrdered;

  /** Symbolic IndexFactor Graph
   *  \nosubgrouping
   */
  class SymbolicFactorGraphOrdered: public FactorGraphOrdered<IndexFactorOrdered> {

  public:

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicFactorGraphOrdered() {
    }

    /** Construct from a BayesNet */
    GTSAM_EXPORT SymbolicFactorGraphOrdered(const SymbolicBayesNetOrdered& bayesNet);

    /** Construct from a BayesTree */
    GTSAM_EXPORT SymbolicFactorGraphOrdered(const SymbolicBayesTreeOrdered& bayesTree);

    /**
     * Construct from a factor graph of any type
     */
    template<class FACTOR>
    SymbolicFactorGraphOrdered(const FactorGraphOrdered<FACTOR>& fg);
    
    /** Eliminate the first \c n frontal variables, returning the resulting
     * conditional and remaining factor graph - this is very inefficient for
     * eliminating all variables, to do that use EliminationTree or
     * JunctionTree.  Note that this version simply calls
     * FactorGraph<IndexFactor>::eliminateFrontals with EliminateSymbolic
     * as the eliminate function argument.
     */
    GTSAM_EXPORT std::pair<sharedConditional, SymbolicFactorGraphOrdered> eliminateFrontals(size_t nFrontals) const;
            
    /** Factor the factor graph into a conditional and a remaining factor graph.
     * Given the factor graph \f$ f(X) \f$, and \c variables to factorize out
     * \f$ V \f$, this function factorizes into \f$ f(X) = f(V;Y)f(Y) \f$, where
     * \f$ Y := X\V \f$ are the remaining variables.  If \f$ f(X) = p(X) \f$ is
     * a probability density or likelihood, the factorization produces a
     * conditional probability density and a marginal \f$ p(X) = p(V|Y)p(Y) \f$.
     *
     * For efficiency, this function treats the variables to eliminate
     * \c variables as fully-connected, so produces a dense (fully-connected)
     * conditional on all of the variables in \c variables, instead of a sparse
     * BayesNet.  If the variables are not fully-connected, it is more efficient
     * to sequentially factorize multiple times.
     * Note that this version simply calls
     * FactorGraph<GaussianFactor>::eliminate with EliminateSymbolic as the eliminate
     * function argument.
     */
    GTSAM_EXPORT std::pair<sharedConditional, SymbolicFactorGraphOrdered> eliminate(const std::vector<Index>& variables) const;

    /** Eliminate a single variable, by calling SymbolicFactorGraph::eliminate. */
    GTSAM_EXPORT std::pair<sharedConditional, SymbolicFactorGraphOrdered> eliminateOne(Index variable) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    GTSAM_EXPORT FastSet<Index> keys() const;

    /// @}
    /// @name Advanced Interface
    /// @{

    /** Push back unary factor */
    GTSAM_EXPORT void push_factor(Index key);

    /** Push back binary factor */
    GTSAM_EXPORT void push_factor(Index key1, Index key2);

    /** Push back ternary factor */
    GTSAM_EXPORT void push_factor(Index key1, Index key2, Index key3);

    /** Push back 4-way factor */
    GTSAM_EXPORT void push_factor(Index key1, Index key2, Index key3, Index key4);

    /** Permute the variables in the factors */
    GTSAM_EXPORT void permuteWithInverse(const Permutation& inversePermutation);

    /** Apply a reduction, which is a remapping of variable indices. */
    GTSAM_EXPORT void reduceWithInverse(const internal::Reduction& inverseReduction);

  };

  /** Create a combined joint factor (new style for EliminationTree). */
  GTSAM_EXPORT IndexFactorOrdered::shared_ptr CombineSymbolic(const FactorGraphOrdered<IndexFactorOrdered>& factors,
    const FastMap<Index, std::vector<Index> >& variableSlots);

  /**
   * CombineAndEliminate provides symbolic elimination.
   * Combine and eliminate can also be called separately, but for this and
   * derived classes calling them separately generally does extra work.
   */
  GTSAM_EXPORT std::pair<boost::shared_ptr<IndexConditionalOrdered>, boost::shared_ptr<IndexFactorOrdered> >
  EliminateSymbolic(const FactorGraphOrdered<IndexFactorOrdered>&, size_t nrFrontals = 1);

  /// @}

  /** Template function implementation */
  template<class FACTOR>
  SymbolicFactorGraphOrdered::SymbolicFactorGraphOrdered(const FactorGraphOrdered<FACTOR>& fg) {
    for (size_t i = 0; i < fg.size(); i++) {
      if (fg[i]) {
        IndexFactorOrdered::shared_ptr factor(new IndexFactorOrdered(*fg[i]));
        push_back(factor);
      } else
        push_back(IndexFactorOrdered::shared_ptr());
    }
  }

} // namespace gtsam
