/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteEliminationTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>

namespace gtsam {

  // Instantiate base class
  template class EliminationTree<DiscreteBayesNet, DiscreteFactorGraph>;

  /* ************************************************************************* */
  DiscreteEliminationTree::DiscreteEliminationTree(
    const DiscreteFactorGraph& factorGraph, const VariableIndex& structure,
    const Ordering& order) :
  Base(factorGraph, structure, order) {}

  /* ************************************************************************* */
  DiscreteEliminationTree::DiscreteEliminationTree(
    const DiscreteFactorGraph& factorGraph, const Ordering& order) :
  Base(factorGraph, order) {}

  /* ************************************************************************* */
  bool DiscreteEliminationTree::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

}
