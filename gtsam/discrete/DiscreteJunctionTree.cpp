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

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/inference/JunctionTree-inst.h>

namespace gtsam {

// Instantiate base classes
template class EliminatableClusterTree<DiscreteBayesTree, DiscreteFactorGraph>;
template class JunctionTree<DiscreteBayesTree, DiscreteFactorGraph>;

/* ************************************************************************* */
DiscreteJunctionTree::DiscreteJunctionTree(
    const DiscreteEliminationTree& eliminationTree)
    : Base(eliminationTree) {}
/* ************************************************************************* */

void DiscreteJunctionTree::print(const std::string& s,
                                 const KeyFormatter& keyFormatter) const {
  auto visitor = [&keyFormatter](
                     const std::shared_ptr<DiscreteJunctionTree::Cluster>& node,
                     const std::string& parentString) {
    // Print the current node
    node->print(parentString + "-", keyFormatter);
    node->factors.print(parentString + "-", keyFormatter);
    std::cout << std::endl;
    return parentString + "| ";  // Increment the indentation
  };
  std::string parentString = s;
  treeTraversal::DepthFirstForest(*this, parentString, visitor);
}

}  // namespace gtsam
