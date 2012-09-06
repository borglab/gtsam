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
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/base/DSFVector.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <list>

using namespace std;

namespace gtsam {

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &gfg, const Parameters &parameters)
  : parameters_(parameters)
{
  initialize(gfg);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph::shared_ptr &jfg, const Parameters &parameters)
  : parameters_(parameters)
{
  initialize(*jfg);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph &Ab1, const GaussianFactorGraph &Ab2, const Parameters &parameters)
  : parameters_(parameters) {

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(Ab1)->eliminate(&EliminateQR);
  initialize(Rc1, boost::make_shared<GaussianFactorGraph>(Ab2));
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianFactorGraph::shared_ptr &Ab1,
    const GaussianFactorGraph::shared_ptr &Ab2, const Parameters &parameters)
  : parameters_(parameters) {

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(*Ab1)->eliminate(&EliminateQR);
  initialize(Rc1, Ab2);
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianBayesNet::shared_ptr &Rc1, const GaussianFactorGraph &Ab2,
    const Parameters &parameters) : parameters_(parameters)
{
  initialize(Rc1, boost::make_shared<GaussianFactorGraph>(Ab2));
}

/**************************************************************************************************/
SubgraphSolver::SubgraphSolver(const GaussianBayesNet::shared_ptr &Rc1,
    const GaussianFactorGraph::shared_ptr &Ab2, const Parameters &parameters) : parameters_(parameters)
{
  initialize(Rc1, Ab2);
}

VectorValues SubgraphSolver::optimize() {
  VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues, Errors>(*pc_, pc_->zero(), parameters_);
  return pc_->x(ybar);
}

void SubgraphSolver::initialize(const GaussianFactorGraph &jfg)
{
  GaussianFactorGraph::shared_ptr Ab1 = boost::make_shared<GaussianFactorGraph>(),
                                  Ab2 = boost::make_shared<GaussianFactorGraph>();

  boost::tie(Ab1, Ab2) = splitGraph(jfg) ;
  if (parameters_.verbosity())
    cout << "Split A into (A1) " << Ab1->size() << " and (A2) " << Ab2->size() << " factors" << endl;

  GaussianBayesNet::shared_ptr Rc1 = EliminationTree<GaussianFactor>::Create(*Ab1)->eliminate(&EliminateQR);
  VectorValues::shared_ptr xbar(new VectorValues(gtsam::optimize(*Rc1)));
  pc_ = boost::make_shared<SubgraphPreconditioner>(Ab2, Rc1, xbar);
}

void SubgraphSolver::initialize(const GaussianBayesNet::shared_ptr &Rc1, const GaussianFactorGraph::shared_ptr &Ab2)
{
  VectorValues::shared_ptr xbar(new VectorValues(gtsam::optimize(*Rc1)));
  pc_ = boost::make_shared<SubgraphPreconditioner>(Ab2, Rc1, xbar);
}

boost::tuple<GaussianFactorGraph::shared_ptr, GaussianFactorGraph::shared_ptr>
SubgraphSolver::splitGraph(const GaussianFactorGraph &jfg) {

  const VariableIndex index(jfg);
  const size_t n = index.size();
  DSFVector D(n);

  GaussianFactorGraph::shared_ptr At(new GaussianFactorGraph());
  GaussianFactorGraph::shared_ptr Ac( new GaussianFactorGraph());

  size_t t = 0;
  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, jfg ) {

    if ( gf->keys().size() > 2 ) {
      throw runtime_error("SubgraphSolver::splitGraph the graph is not simple, sanity check failed ");
    }

    bool augment = false ;

    /* check whether this factor should be augmented to the "tree" graph */
    if ( gf->keys().size() == 1 ) augment = true;
    else {
      const Index u = gf->keys()[0], v = gf->keys()[1],
                  u_root = D.findSet(u), v_root = D.findSet(v);
      if ( u_root != v_root ) {
        t++; augment = true ;
        D.makeUnionInPlace(u_root, v_root);
      }
    }
    if ( augment ) At->push_back(gf);
    else Ac->push_back(gf);
  }

  return boost::tie(At, Ac);
}

} // \namespace gtsam
