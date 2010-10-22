/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianMultifrontalSolver.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Oct 21, 2010
 */

#include <gtsam/linear/GaussianMultifrontalSolver.h>

#include <gtsam/inference/GenericMultifrontalSolver-inl.h>

namespace gtsam {

/* ************************************************************************* */
GaussianMultifrontalSolver::GaussianMultifrontalSolver(const FactorGraph<GaussianFactor>& factorGraph) :
    Base(factorGraph) {}

/* ************************************************************************* */
BayesTree<GaussianConditional>::shared_ptr GaussianMultifrontalSolver::eliminate() const {
  return Base::eliminate();
}

/* ************************************************************************* */
VectorValues::shared_ptr GaussianMultifrontalSolver::optimize() const {
  return VectorValues::shared_ptr(new VectorValues(junctionTree_.optimize()));
}

/* ************************************************************************* */
GaussianFactor::shared_ptr GaussianMultifrontalSolver::marginal(Index j) const {
  return Base::marginal(j);
}

}
