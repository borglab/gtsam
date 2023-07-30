/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SubgraphSolver.cpp
 * @brief  Subgraph Solver from IROS 2010
 * @date   2010
 * @author Frank Dellaert
 * @author Yong Dian Jian
 */

#include <gtsam/linear/SubgraphSolver.h>

#include <gtsam/linear/SubgraphBuilder.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphPreconditioner.h>

using namespace std;

namespace gtsam {

/**************************************************************************************************/
// Just taking system [A|b]
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &Ab,
    const Parameters &parameters, const Ordering& ordering) :
    parameters_(parameters) {
  const auto [Ab1, Ab2] = splitGraph(Ab);
  if (parameters_.verbosity())
    cout << "Split A into (A1) " << Ab1.size() << " and (A2) " << Ab2.size()
         << " factors" << endl;

  auto Rc1 = *Ab1.eliminateSequential(ordering, EliminateQR);
  auto xbar = Rc1.optimize();
  pc_ = std::make_shared<SubgraphPreconditioner>(Ab2, Rc1, xbar);
}

/**************************************************************************************************/
// Taking eliminated tree [R1|c] and constraint graph [A2|b2]
SubgraphSolver::SubgraphSolver(const GaussianBayesNet &Rc1,
                               const GaussianFactorGraph &Ab2,
                               const Parameters &parameters)
    : parameters_(parameters) {
  auto xbar = Rc1.optimize();
  pc_ = std::make_shared<SubgraphPreconditioner>(Ab2, Rc1, xbar);
}

/**************************************************************************************************/
// Taking subgraphs [A1|b1] and [A2|b2]
// delegate up
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &Ab1,
                               const GaussianFactorGraph &Ab2,
                               const Parameters &parameters,
                               const Ordering &ordering)
    : SubgraphSolver(*Ab1.eliminateSequential(ordering, EliminateQR), Ab2,
                     parameters) {}

/**************************************************************************************************/
VectorValues SubgraphSolver::optimize() const {
  VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues,
      Errors>(*pc_, pc_->zero(), parameters_);
  return pc_->x(ybar);
}

VectorValues SubgraphSolver::optimize(const GaussianFactorGraph &gfg,
    const KeyInfo &keyInfo, const map<Key, Vector> &lambda,
    const VectorValues &initial) {
  return VectorValues();
}
/**************************************************************************************************/
pair<GaussianFactorGraph, GaussianFactorGraph> //
SubgraphSolver::splitGraph(const GaussianFactorGraph &factorGraph) {

  /* identify the subgraph structure */
  const SubgraphBuilder builder(parameters_.builderParams);
  auto subgraph = builder(factorGraph);

  /* build factor subgraph */
  return splitFactorGraph(factorGraph, subgraph);
}

/****************************************************************************/

} // \namespace gtsam
