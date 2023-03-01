/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicEliminationTree.h
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtsam {

  class GTSAM_EXPORT SymbolicEliminationTree :
    public EliminationTree<SymbolicBayesNet, SymbolicFactorGraph>
  {
  public:
    typedef EliminationTree<SymbolicBayesNet, SymbolicFactorGraph> Base; ///< Base class
    typedef SymbolicEliminationTree This; ///< This class
    typedef std::shared_ptr<This> shared_ptr; ///< Shared pointer to this class

    /** Build the elimination tree of a factor graph using pre-computed column structure.
     *  @param factorGraph The factor graph for which to build the elimination tree
     *  @param structure The set of factors involving each variable.  If this is not precomputed,
     *         you can call the Create(const FactorGraph<DERIVEDFACTOR>&) named constructor instead.
     *  @return The elimination tree */
    SymbolicEliminationTree(const SymbolicFactorGraph& factorGraph,
      const VariableIndex& structure, const Ordering& order);

    /** Build the elimination tree of a factor graph.  Note that this has to compute the column
     *  structure as a VariableIndex, so if you already have this precomputed, use the other
     *  constructor instead.
     *  @param factorGraph The factor graph for which to build the elimination tree */
    SymbolicEliminationTree(const SymbolicFactorGraph& factorGraph,
      const Ordering& order);

    /** Test whether the tree is equal to another */
    bool equals(const This& other, double tol = 1e-9) const;

  private:

    /** Private default constructor for testing */
    SymbolicEliminationTree() {}

    friend class ::EliminationTreeTester;

  };

/// traits
template<>
struct traits<SymbolicEliminationTree> : public Testable<SymbolicEliminationTree> {
};

} //\ namespace gtsam
