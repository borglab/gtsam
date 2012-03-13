/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree.h
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

typedef BayesTree<GaussianConditional> GaussianBayesTree;

/// optimize the BayesTree, starting from the root
template<class CLIQUE>
VectorValues optimize(const BayesTree<GaussianConditional, CLIQUE>& bayesTree);

/// recursively optimize this conditional and all subtrees
template<class CLIQUE>
void optimizeInPlace(const BayesTree<GaussianConditional, CLIQUE>& clique, VectorValues& result);

}

#include <gtsam/linear/GaussianBayesTree-inl.h>

