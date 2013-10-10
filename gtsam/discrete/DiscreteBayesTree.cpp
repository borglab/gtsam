/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DiscreteBayesTree.cpp
 * @brief   Discrete Bayes Tree, the result of eliminating a DiscreteJunctionTree
 * @brief   DiscreteBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteBayesNet.h>

namespace gtsam {

  // Instantiate base class
  template class BayesTreeCliqueBase<DiscreteBayesTreeClique, DiscreteFactorGraph>;
  template class BayesTree<DiscreteBayesTreeClique>;


  /* ************************************************************************* */
  bool DiscreteBayesTree::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }

} // \namespace gtsam




