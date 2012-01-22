/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <boost/foreach.hpp>

#include <gtsam/linear/iterative-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/EliminationTree.h>

using namespace std;

namespace gtsam {

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

} // \namespace gtsam
