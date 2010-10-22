/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericMultifrontalSolver-inl.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/GenericMultifrontalSolver.h>
#include <gtsam/inference/FactorBase-inl.h>
#include <gtsam/inference/JunctionTree-inl.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/inference-inl.h>

#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
template<class FACTOR, class JUNCTIONTREE>
GenericMultifrontalSolver<FACTOR, JUNCTIONTREE>::GenericMultifrontalSolver(const FactorGraph<FACTOR>& factorGraph) :
    junctionTree_(factorGraph) {}

/* ************************************************************************* */
template<class FACTOR, class JUNCTIONTREE>
typename JUNCTIONTREE::BayesTree::shared_ptr
GenericMultifrontalSolver<FACTOR, JUNCTIONTREE>::eliminate() const {
  typename JUNCTIONTREE::BayesTree::shared_ptr bayesTree(new typename JUNCTIONTREE::BayesTree);
  bayesTree->insert(junctionTree_.eliminate());
  return bayesTree;
}

/* ************************************************************************* */
template<class FACTOR, class JUNCTIONTREE>
typename FACTOR::shared_ptr GenericMultifrontalSolver<FACTOR, JUNCTIONTREE>::marginal(Index j) const {
  return eliminate()->marginal(j);
}

}


