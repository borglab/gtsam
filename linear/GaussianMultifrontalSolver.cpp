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
GaussianMultifrontalSolver::shared_ptr
GaussianMultifrontalSolver::Create(const FactorGraph<GaussianFactor>& factorGraph) {
  return shared_ptr(new GaussianMultifrontalSolver(factorGraph));
}

/* ************************************************************************* */
GaussianMultifrontalSolver::shared_ptr
GaussianMultifrontalSolver::update(const FactorGraph<GaussianFactor>& factorGraph) const {
  // We do not yet have code written to update the junction tree, so we just
  // create a new solver.
  return Create(factorGraph);
}

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

/* ************************************************************************* */
std::pair<Vector, Matrix> GaussianMultifrontalSolver::marginalStandard(Index j) const {
	GaussianConditional::shared_ptr conditional = Base::marginal(j)->eliminateFirst();
	Matrix R = conditional->get_R();
	return make_pair(conditional->get_d(), inverse(trans(R)*R));
}

}
