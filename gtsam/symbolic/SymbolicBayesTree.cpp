/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesTree.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>

namespace gtsam {

  // Instantiate base classes
  template class BayesTreeCliqueBase<SymbolicBayesTreeClique, SymbolicFactorGraph>;
  template class BayesTree<SymbolicBayesTreeClique>;

  /* ************************************************************************* */\
  bool SymbolicBayesTree::equals(const This& other, double tol /* = 1e-9 */) const
  {
    return Base::equals(other, tol);
  }

}
