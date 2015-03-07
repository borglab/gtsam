/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTree-inl.h
 * @brief   Gaussian Bayes Tree, the result of eliminating a GaussianJunctionTree
 * @brief   GaussianBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <boost/foreach.hpp>

#include <gtsam/linear/GaussianBayesTree.h> // Only to help Eclipse

namespace gtsam {

/* ************************************************************************* */
namespace internal {
template<class BAYESTREE>
void optimizeInPlace(const typename BAYESTREE::sharedClique& clique, VectorValues& result) {
  // parents are assumed to already be solved and available in result
  clique->conditional()->solveInPlace(result);

  // starting from the root, call optimize on each conditional
  BOOST_FOREACH(const typename BAYESTREE::sharedClique& child, clique->children_)
    optimizeInPlace<BAYESTREE>(child, result);
}
}

}
