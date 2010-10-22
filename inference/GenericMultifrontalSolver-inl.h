/**
 * @file    GenericMultifrontalSolver-inl.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#pragma once

#include <gtsam/inference/GenericMultifrontalSolver.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/JunctionTree-inl.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/inference-inl.h>

#include <boost/foreach.hpp>

namespace gtsam {

/* ************************************************************************* */
template<class FACTOR>
GenericMultifrontalSolver<FACTOR>::GenericMultifrontalSolver(const FactorGraph<FACTOR>& factorGraph) :
    junctionTree_(new JunctionTree<FactorGraph<FACTOR> >(factorGraph)) {}

/* ************************************************************************* */
template<class FACTOR>
typename BayesTree<typename FACTOR::Conditional>::shared_ptr GenericMultifrontalSolver<FACTOR>::eliminate() const {
  return junctionTree_->eliminate();
}

/* ************************************************************************* */
template<class FACTOR>
typename FACTOR::shared_ptr GenericMultifrontalSolver<FACTOR>::marginal(Index j) const {
  return eliminate()->marginal(j);
}

}


