/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicJunctionTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/JunctionTree-inst.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>

namespace gtsam {

  // Instantiate base class
  template class EliminatableClusterTree<SymbolicBayesTree, SymbolicFactorGraph>;
  template class JunctionTree<SymbolicBayesTree, SymbolicFactorGraph>;

  /* ************************************************************************* */
  SymbolicJunctionTree::SymbolicJunctionTree(
    const SymbolicEliminationTree& eliminationTree) :
  Base(eliminationTree) {}

}
