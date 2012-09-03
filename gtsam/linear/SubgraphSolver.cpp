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
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/linear/JacobianFactorGraph.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/EliminationTree.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <list>

using namespace std;

namespace gtsam {

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &gfg, const Parameters &parameters)
  : parameters_(parameters)
{
  JacobianFactorGraph::shared_ptr jfg = dynamicCastFactors(gfg);
  initialize(*jfg);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const JacobianFactorGraph::shared_ptr &jfg, const Parameters &parameters)
  : parameters_(parameters)
{
  initialize(*jfg);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &Ab1, const GaussianFactorGraph &Ab2, const Parameters &parameters)
  : parameters_(parameters) {

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(Ab1)->eliminate(&EliminateQR);
  JacobianFactorGraph::shared_ptr Ab2Jacobian = dynamicCastFactors(Ab2);
  initialize(Rc1, Ab2Jacobian);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const JacobianFactorGraph::shared_ptr &Ab1,
    const JacobianFactorGraph::shared_ptr &Ab2, const Parameters &parameters)
  : parameters_(parameters) {

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(*Ab1)->eliminate(&EliminateQR);
  initialize(Rc1, Ab2);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianBayesNet::shared_ptr &Rc1, const GaussianFactorGraph &Ab2,
    const Parameters &parameters) : parameters_(parameters)
{
  JacobianFactorGraph::shared_ptr Ab2Jacobians = dynamicCastFactors(Ab2);
  initialize(Rc1, Ab2Jacobians);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianBayesNet::shared_ptr &Rc1,
    const JacobianFactorGraph::shared_ptr &Ab2, const Parameters &parameters) : parameters_(parameters)
{
  initialize(Rc1, Ab2);
}

VectorValues SubgraphSolver::optimize() {
  VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues, Errors>(*pc_, pc_->zero(), parameters_);
  return pc_->x(ybar);
}

void SubgraphSolver::initialize(const JacobianFactorGraph &jfg)
{
  JacobianFactorGraph::shared_ptr Ab1 = boost::make_shared<JacobianFactorGraph>(),
                                  Ab2 = boost::make_shared<JacobianFactorGraph>();

  boost::tie(Ab1, Ab2) = splitGraph(jfg) ;
  if (parameters_.verbosity())
    cout << "Split A into (A1) " << Ab1->size() << " and (A2) " << Ab2->size() << " factors" << endl;

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(*Ab1)->eliminate(&EliminateQR);
  VectorValues::shared_ptr xbar(new VectorValues(gtsam::optimize(*Rc1)));
  JacobianFactorGraph::shared_ptr Ab2Jacobians = convertToJacobianFactorGraph(*Ab2);
  pc_ = boost::make_shared<SubgraphPreconditioner>(Ab2Jacobians, Rc1, xbar);
}

void SubgraphSolver::initialize(const GaussianBayesNet::shared_ptr &Rc1, const JacobianFactorGraph::shared_ptr &Ab2)
{
  VectorValues::shared_ptr xbar(new VectorValues(gtsam::optimize(*Rc1)));
  pc_ = boost::make_shared<SubgraphPreconditioner>(Ab2, Rc1, xbar);
}

boost::tuple<JacobianFactorGraph::shared_ptr, JacobianFactorGraph::shared_ptr>
SubgraphSolver::splitGraph(const JacobianFactorGraph &jfg) {

  const VariableIndex index(jfg);
  const size_t n = index.size(), m = jfg.size();
  DisjointSet D(n) ;

  JacobianFactorGraph::shared_ptr At(new JacobianFactorGraph());
  JacobianFactorGraph::shared_ptr Ac( new JacobianFactorGraph());

  size_t t = 0;
  BOOST_FOREACH ( const JacobianFactor::shared_ptr &jf, jfg ) {

    if ( jf->keys().size() > 2 ) {
      throw runtime_error("SubgraphSolver::splitGraph the graph is not simple, sanity check failed ");
    }

    bool augment = false ;

    /* check whether this factor should be augmented to the "tree" graph */
    if ( jf->keys().size() == 1 ) augment = true;
    else {
      const Index u = jf->keys()[0], v = jf->keys()[1],
                  u_root = D.find(u), v_root = D.find(v);
      if ( u_root != v_root ) {
        t++; augment = true ;
        D.makeUnion(u_root, v_root);
      }
    }
    if ( augment ) At->push_back(jf);
    else Ac->push_back(jf);
  }

  return boost::tie(At, Ac);
}


SubgraphSolver::DisjointSet::DisjointSet(const size_t n):n_(n),rank_(n,1),parent_(n) {
  for ( Index i = 0 ; i < n ; ++i ) parent_[i] = i ;
}

Index SubgraphSolver::DisjointSet::makeUnion(const Index &u, const Index &v) {

  Index u_root = find(u), v_root = find(v) ;
  Index u_rank = rank(u), v_rank = rank(v) ;

  if ( u_root != v_root ) {
    if ( v_rank > u_rank ) {
      parent_[u_root] = v_root ;
      rank_[v_root] += rank_[u_root] ;
      return v_root ;
    }
    else {
      parent_[v_root] = u_root ;
      rank_[u_root] += rank_[v_root] ;
      return u_root ;
    }
  }
  return u_root ;
}

Index SubgraphSolver::DisjointSet::find(const Index &u) {
  vector<Index> path ;
  Index x = u;
  Index x_root = parent_[x] ;

  // find the root, and keep the vertices along the path
  while ( x != x_root ) {
    path.push_back(x) ;
    x = x_root ;
    x_root = parent_[x] ;
  }

  // path compression
  BOOST_FOREACH(const Index &i, path) {
    rank_[i] = 1 ;
    parent_[i] = x_root ;
  }

  return x_root ;
}




} // \namespace gtsam
