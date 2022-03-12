/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridJunctionTree.cpp
 * @date Mar 11, 2022
 * @author Fan Jiang
 */

#include <gtsam/inference/JunctionTree-inst.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/hybrid/HybridEliminationTree.h>

namespace gtsam {

// Instantiate base classes
template class EliminatableClusterTree<HybridBayesTree, HybridFactorGraph>;
template class JunctionTree<HybridBayesTree, HybridFactorGraph>;

/* ************************************************************************* */
HybridJunctionTree::HybridJunctionTree(
    const HybridEliminationTree& eliminationTree) :
    Base(eliminationTree) {}

}
