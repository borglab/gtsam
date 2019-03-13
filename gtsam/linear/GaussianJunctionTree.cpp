/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianJunctionTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/JunctionTree-inst.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>

namespace gtsam {

  // Instantiate base classes
  template class EliminatableClusterTree<GaussianBayesTree, GaussianFactorGraph>;
  template class JunctionTree<GaussianBayesTree, GaussianFactorGraph>;

  /* ************************************************************************* */
  GaussianJunctionTree::GaussianJunctionTree(
    const GaussianEliminationTree& eliminationTree) :
  Base(eliminationTree) {}

}
