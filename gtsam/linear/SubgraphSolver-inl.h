/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/inference/EliminationTree-inl.h>


using namespace std;

namespace gtsam {

template<class GRAPH, class LINEAR, class VALUES>
typename SubgraphSolver<GRAPH,LINEAR,VALUES>::shared_ptr
SubgraphSolver<GRAPH,LINEAR,VALUES>::update(const LINEAR &graph) const {

	shared_linear Ab1 = boost::make_shared<LINEAR>(),
				  Ab2 = boost::make_shared<LINEAR>();

	if (parameters_->verbosity()) cout << "split the graph ...";
	graph.split(pairs_, *Ab1, *Ab2) ;
	if (parameters_->verbosity()) cout << ",with " << Ab1->size() << " and " << Ab2->size() << " factors" << endl;

	//	// Add a HardConstraint to the root, otherwise the root will be singular
	//	Key root = keys.back();
	//	T_.addHardConstraint(root, theta0[root]);
	//
	//	// compose the approximate solution
	//	theta_bar_ = composePoses<GRAPH, Constraint, Pose, Values> (T_, tree, theta0[root]);

	LINEAR sacrificialAb1 = *Ab1; // duplicate !!!!!
	SubgraphPreconditioner::sharedBayesNet Rc1 = EliminationTree<GaussianFactor>::Create(sacrificialAb1)->eliminate();
	SubgraphPreconditioner::sharedValues xbar = gtsam::optimize_(*Rc1);

	shared_preconditioner pc = boost::make_shared<SubgraphPreconditioner>(Ab1,Ab2,Rc1,xbar);
	return boost::make_shared<SubgraphSolver>(ordering_, pairs_, pc, parameters_) ;
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
