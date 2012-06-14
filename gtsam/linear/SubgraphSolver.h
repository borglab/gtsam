/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/linear/ConjugateGradientSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphPreconditioner.h>

#include <boost/make_shared.hpp>

namespace gtsam {

class SubgraphSolverParameters : public ConjugateGradientParameters {
public:
  typedef ConjugateGradientParameters Base;
  SubgraphSolverParameters() : Base() {}
};

/**
 * A linear system solver using subgraph preconditioning conjugate gradient
 */

class SubgraphSolver : public IterativeSolver {

public:

  typedef SubgraphSolverParameters Parameters;

protected:

	Parameters parameters_;
	SubgraphPreconditioner::shared_ptr pc_;  ///< preconditioner object

public:

	SubgraphSolver(const GaussianFactorGraph &gfg, const Parameters &parameters);
  virtual ~SubgraphSolver() {}
  virtual VectorValues optimize () ;

protected:

  boost::tuple<GaussianFactorGraph::shared_ptr, GaussianFactorGraph::shared_ptr>
  splitGraph(const GaussianFactorGraph &gfg) ;
};

} // namespace gtsam


