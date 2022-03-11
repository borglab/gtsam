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

#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/hybrid/HybridEliminationTree.h>

namespace gtsam {

// Instantiate base class
template class EliminationTree<HybridBayesNet, HybridFactorGraph>;

/* ************************************************************************* */
HybridEliminationTree::HybridEliminationTree(
    const HybridFactorGraph& factorGraph, const VariableIndex& structure,
    const Ordering& order) :
    Base(factorGraph, structure, order) {}

/* ************************************************************************* */
HybridEliminationTree::HybridEliminationTree(
    const HybridFactorGraph& factorGraph, const Ordering& order) :
    Base(factorGraph, order) {}

/* ************************************************************************* */
bool HybridEliminationTree::equals(const This& other, double tol) const
{
  return Base::equals(other, tol);
}

}
