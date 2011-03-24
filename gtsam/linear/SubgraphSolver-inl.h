/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <map>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/inference/EliminationTree-inl.h>

using namespace std;

namespace gtsam {

/* split the gaussian factor graph Ab into Ab1 and Ab2 according to the map */
bool split(const std::map<Index, Index> &M,
           const GaussianFactorGraph &Ab,
           GaussianFactorGraph &Ab1,
           GaussianFactorGraph &Ab2) {

  Ab1 = GaussianFactorGraph();
  Ab2 = GaussianFactorGraph();

  for ( size_t i = 0 ; i < Ab.size() ; ++i ) {

    boost::shared_ptr<GaussianFactor> factor = Ab[i] ;

    if (factor->keys().size() > 2)
        throw(invalid_argument("split: only support factors with at most two keys"));
    if (factor->keys().size() == 1) {
        Ab1.push_back(factor);
        Ab2.push_back(factor);
        continue;
    }
    Index key1 = factor->keys()[0];
    Index key2 = factor->keys()[1];

    if ((M.find(key1) != M.end() && M.find(key1)->second == key2) ||
        (M.find(key2) != M.end() && M.find(key2)->second == key1))
        Ab1.push_back(factor);
    else
        Ab2.push_back(factor);
  }
  return true ;
}


template<class GRAPH, class LINEAR, class VALUES>
void SubgraphSolver<GRAPH,LINEAR,VALUES>::replaceFactors(const typename LINEAR::shared_ptr &graph) {

	GaussianFactorGraph::shared_ptr Ab1 = boost::make_shared<GaussianFactorGraph>();
	GaussianFactorGraph::shared_ptr Ab2 = boost::make_shared<GaussianFactorGraph>();

	if (parameters_->verbosity()) cout << "split the graph ...";
	split(pairs_, *graph, *Ab1, *Ab2) ;
	if (parameters_->verbosity()) cout << ",with " << Ab1->size() << " and " << Ab2->size() << " factors" << endl;

	//	// Add a HardConstraint to the root, otherwise the root will be singular
	//	Key root = keys.back();
	//	T_.addHardConstraint(root, theta0[root]);
	//
	//	// compose the approximate solution
	//	theta_bar_ = composePoses<GRAPH, Constraint, Pose, Values> (T_, tree, theta0[root]);

	LINEAR sacrificialAb1 = *Ab1; // duplicate !!!!!
	SubgraphPreconditioner::sharedBayesNet Rc1 =
			EliminationTree<GaussianFactor>::Create(sacrificialAb1)->eliminate(&EliminateQR);
	SubgraphPreconditioner::sharedValues xbar = gtsam::optimize_(*Rc1);

	pc_ = boost::make_shared<SubgraphPreconditioner>(
	    Ab1->dynamicCastFactors<FactorGraph<JacobianFactor> >(), Ab2->dynamicCastFactors<FactorGraph<JacobianFactor> >(),Rc1,xbar);
}

template<class GRAPH, class LINEAR, class VALUES>
VectorValues::shared_ptr SubgraphSolver<GRAPH,LINEAR,VALUES>::optimize() const {

	// preconditioned conjugate gradient
	VectorValues zeros = pc_->zero();
	VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues, Errors>
		(*pc_, zeros, *parameters_);

	boost::shared_ptr<VectorValues> xbar = boost::make_shared<VectorValues>() ;
	*xbar = pc_->x(ybar);
	return xbar;
}

template<class GRAPH, class LINEAR, class VALUES>
void SubgraphSolver<GRAPH,LINEAR,VALUES>::initialize(const GRAPH& G, const VALUES& theta0) {
		// generate spanning tree
		PredecessorMap<Key> tree_ = gtsam::findMinimumSpanningTree<GRAPH, Key, Constraint>(G);

		// make the ordering
		list<Key> keys = predecessorMap2Keys(tree_);
		ordering_ = boost::make_shared<Ordering>(list<Symbol>(keys.begin(), keys.end()));

		// build factor pairs
		pairs_.clear();
		typedef pair<Key,Key> EG ;
		BOOST_FOREACH( const EG &eg, tree_ ) {
			Symbol key1 = Symbol(eg.first),
				   key2 = Symbol(eg.second) ;
			pairs_.insert(pair<Index, Index>((*ordering_)[key1], (*ordering_)[key2])) ;
		}
	}









}
