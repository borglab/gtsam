/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianEliminationTree.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/linear/internal/GaussianElimination.h>
#include <gtsam/linear/internal/EliminationTreeFactors.h>
#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/linear/GaussianEliminationTree.h>

namespace gtsam {

  // Instantiate base class
  template class EliminationTree<GaussianBayesNet, GaussianFactorGraph>;

  /* ************************************************************************* */
  GaussianEliminationTree::GaussianEliminationTree(
    const GaussianFactorGraph& factorGraph, const VariableIndex& structure,
    const Ordering& order) :
  Base(factorGraph, structure, order) {}

  /* ************************************************************************* */
  GaussianEliminationTree::GaussianEliminationTree(
    const GaussianFactorGraph& factorGraph, const Ordering& order) :
  Base(factorGraph, order) {}

  /* ************************************************************************* */
  bool GaussianEliminationTree::equals(const This& other, double tol) const
  {
    return Base::equals(other, tol);
  }


std::pair<std::shared_ptr<GaussianBayesNet>, std::shared_ptr<GaussianFactorGraph>>
GaussianEliminationTree::eliminate(const Eliminate& function) const {
  const internal::GaussianElimination operation(
      internal::eliminationTreeFactors(*this), function);
  return Base::eliminate(operation.function());
}
}
