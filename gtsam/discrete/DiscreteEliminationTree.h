/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteEliminationTree.h
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtsam {

  /**
   * @brief Elimination tree for discrete factors.
   * @ingroup discrete
   */
  class GTSAM_EXPORT DiscreteEliminationTree :
    public EliminationTree<DiscreteBayesNet, DiscreteFactorGraph>
  {
  public:
    typedef EliminationTree<DiscreteBayesNet, DiscreteFactorGraph> Base; ///< Base class
    typedef DiscreteEliminationTree This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class

    /**
    * Build the elimination tree of a factor graph using pre-computed column structure.
    * @param factorGraph The factor graph for which to build the elimination tree
    * @param structure The set of factors involving each variable.  If this is not
    * precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
    * named constructor instead.
    * @return The elimination tree
    */
    DiscreteEliminationTree(const DiscreteFactorGraph& factorGraph,
      const VariableIndex& structure, const Ordering& order);

    /** Build the elimination tree of a factor graph.  Note that this has to compute the column
    * structure as a VariableIndex, so if you already have this precomputed, use the other
    * constructor instead.
    * @param factorGraph The factor graph for which to build the elimination tree
    */
    DiscreteEliminationTree(const DiscreteFactorGraph& factorGraph,
      const Ordering& order);

    /** Test whether the tree is equal to another */
    bool equals(const This& other, double tol = 1e-9) const;

  private:

    friend class ::EliminationTreeTester;

  };

}
