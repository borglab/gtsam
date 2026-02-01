/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicEliminationTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>

namespace gtsam {

  // Instantiate base class
  template class EliminationTree<SymbolicBayesNet, SymbolicFactorGraph>;

  /* ************************************************************************* */
  SymbolicEliminationTree::SymbolicEliminationTree(
    const SymbolicFactorGraph& factorGraph, const VariableIndex& structure,
    const Ordering& order) :
  Base(factorGraph, structure, order) {}

  /* ************************************************************************* */
  SymbolicEliminationTree::SymbolicEliminationTree(
    const SymbolicFactorGraph& factorGraph, const Ordering& order) :
  Base(factorGraph, order) {}

  /* ************************************************************************* */
  bool SymbolicEliminationTree::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

}
