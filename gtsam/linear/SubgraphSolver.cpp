/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactorGraph.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/EliminationTree.h>

#include <gtsam_unstable/linear/iterative-inl.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <list>

using namespace std;

namespace gtsam {

SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &gfg, const Parameters &parameters)
  : parameters_(parameters)
{


  GaussianFactorGraph::shared_ptr Ab1 = boost::make_shared<GaussianFactorGraph>();
  GaussianFactorGraph::shared_ptr Ab2 = boost::make_shared<GaussianFactorGraph>();

  boost::tie(Ab1, Ab2) = splitGraph(gfg) ;

  if (parameters_.verbosity())
    cout << ",with " << Ab1->size() << " and " << Ab2->size() << " factors" << endl;

  //  // Add a HardConstraint to the root, otherwise the root will be singular
  //  Key root = keys.back();
  //  T_.addHardConstraint(root, theta0[root]);
  //
  //  // compose the approximate solution
  //  theta_bar_ = composePoses<GRAPH, Constraint, Pose, Values> (T_, tree, theta0[root]);

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(*Ab1)->eliminate(&EliminateQR);
  VectorValues::shared_ptr xbar(new VectorValues(gtsam::optimize(*Rc1)));

  pc_ = boost::make_shared<SubgraphPreconditioner>(
      Ab1->dynamicCastFactors<FactorGraph<JacobianFactor> >(),
      Ab2->dynamicCastFactors<FactorGraph<JacobianFactor> >(),
      Rc1, xbar);
}

VectorValues::shared_ptr SubgraphSolver::optimize() {

  // preconditioned conjugate gradient
  VectorValues zeros = pc_->zero();
  VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues, Errors> (*pc_, zeros, parameters_);

  boost::shared_ptr<VectorValues> xbar = boost::make_shared<VectorValues>() ;
  *xbar = pc_->x(ybar);
  return xbar;
}

boost::tuple<GaussianFactorGraph::shared_ptr, GaussianFactorGraph::shared_ptr>
SubgraphSolver::splitGraph(const GaussianFactorGraph &gfg) {

  VariableIndex index(gfg);
  size_t n = index.size();
  std::vector<bool> connected(n, false);

  GaussianFactorGraph::shared_ptr At(new GaussianFactorGraph());
  GaussianFactorGraph::shared_ptr Ac( new GaussianFactorGraph());

  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, gfg ) {

    bool augment = false ;

    /* check whether this factor should be augmented to the "tree" graph */
    if ( gf->keys().size() == 1 ) augment = true;
    else {
      BOOST_FOREACH ( const Index key, *gf ) {
        if ( connected[key] == false ) {
          augment = true ;
          connected[key] = true;
        }
      }
    }

    if ( augment ) At->push_back(gf);
    else Ac->push_back(gf);
  }

  return boost::tie(At, Ac);
}




} // \namespace gtsam
