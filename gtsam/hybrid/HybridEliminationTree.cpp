/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridEliminationTree.cpp
 * @date Mar 11, 2022
 * @author Fan Jiang
 */

#include <gtsam/hybrid/internal/HybridElimination.h>
#include <gtsam/linear/internal/EliminationTreeFactors.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/inference/EliminationTree-inst.h>

namespace gtsam {

// Instantiate base class
template class EliminationTree<HybridBayesNet, HybridGaussianFactorGraph>;

/* ************************************************************************* */
HybridEliminationTree::HybridEliminationTree(
    const HybridGaussianFactorGraph& factorGraph,
    const VariableIndex& structure, const Ordering& order)
    : Base(factorGraph, structure, order) {}

/* ************************************************************************* */
HybridEliminationTree::HybridEliminationTree(
    const HybridGaussianFactorGraph& factorGraph, const Ordering& order)
    : Base(factorGraph, order) {}

/* ************************************************************************* */
bool HybridEliminationTree::equals(const This& other, double tol) const {
  return Base::equals(other, tol);
}


std::pair<std::shared_ptr<HybridBayesNet>, std::shared_ptr<HybridGaussianFactorGraph>>
HybridEliminationTree::eliminate(const Eliminate& function) const {
  const internal::HybridElimination operation(
      internal::eliminationTreeFactors(*this), function);
  return Base::eliminate(operation.function());
}
}  // namespace gtsam
