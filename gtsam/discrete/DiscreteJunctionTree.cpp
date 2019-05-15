/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteJunctionTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/JunctionTree-inst.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>

namespace gtsam {

  // Instantiate base classes
  template class EliminatableClusterTree<DiscreteBayesTree, DiscreteFactorGraph>;
  template class JunctionTree<DiscreteBayesTree, DiscreteFactorGraph>;

  /* ************************************************************************* */
  DiscreteJunctionTree::DiscreteJunctionTree(
    const DiscreteEliminationTree& eliminationTree) :
  Base(eliminationTree) {}

}
