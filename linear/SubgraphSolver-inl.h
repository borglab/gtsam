/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * SubgraphSolver-inl.h
 *
 *   Created on: Jan 17, 2010
 *       Author: nikai
 *  Description: subgraph preconditioning conjugate gradient solver
 */

#pragma once

#include <boost/tuple/tuple.hpp>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/inference/EliminationTree-inl.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class GRAPH, class VALUES>
	SubgraphSolver<GRAPH, VALUES>::SubgraphSolver(const GRAPH& G, const VALUES& theta0) {
		initialize(G,theta0);
	}

	/* ************************************************************************* */
	template<class GRAPH, class VALUES>
	void SubgraphSolver<GRAPH, VALUES>::initialize(const GRAPH& G, const VALUES& theta0) {

		// generate spanning tree
		PredecessorMap<Key> tree = gtsam::findMinimumSpanningTree<GRAPH, Key, Constraint>(G);

		// split the graph
		if (verbose_) cout << "generating spanning tree and split the graph ...";
		gtsam::split<GRAPH,Key,Constraint>(G, tree, T_, C_) ;
		if (verbose_) cout << ",with " << T_.size() << " and " << C_.size() << " factors" << endl;

		// make the ordering
		list<Key> keys = predecessorMap2Keys(tree);
		ordering_ = boost::shared_ptr<Ordering>(new Ordering(list<Symbol>(keys.begin(), keys.end())));

		// Add a HardConstraint to the root, otherwise the root will be singular
		Key root = keys.back();
		T_.addHardConstraint(root, theta0[root]);

		// compose the approximate solution
		theta_bar_ = composePoses<GRAPH, Constraint, Pose, VALUES> (T_, tree, theta0[root]);
	}

	/* ************************************************************************* */
	template<class GRAPH, class VALUES>
	boost::shared_ptr<SubgraphPreconditioner> SubgraphSolver<GRAPH, VALUES>::linearize(const GRAPH& G, const VALUES& theta_bar) const {
		SubgraphPreconditioner::sharedFG Ab1 = T_.linearize(theta_bar, *ordering_);
		SubgraphPreconditioner::sharedFG Ab2 = C_.linearize(theta_bar, *ordering_);
#ifdef TIMING
		SubgraphPreconditioner::sharedBayesNet Rc1;
		SubgraphPreconditioner::sharedValues xbar;
#else
		GaussianFactorGraph sacrificialAb1 = *Ab1; // duplicate !!!!!
		SubgraphPreconditioner::sharedBayesNet Rc1 = EliminationTree<GaussianFactor>::Create(sacrificialAb1)->eliminate();
		SubgraphPreconditioner::sharedValues xbar = gtsam::optimize_(*Rc1);
#endif
		// TODO: there does not seem to be a good reason to have Ab1_
		// It seems only be used to provide an ordering for creating sparse matrices
		return boost::shared_ptr<SubgraphPreconditioner>(new SubgraphPreconditioner(Ab1, Ab2, Rc1, xbar));
	}

	/* ************************************************************************* */
	template<class GRAPH, class VALUES>
	VectorValues SubgraphSolver<GRAPH, VALUES>::optimize(SubgraphPreconditioner& system) const {
		VectorValues zeros = system.zero();

		// Solve the subgraph PCG
		VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues,
				Errors> (system, zeros, verbose_, epsilon_, epsilon_abs_, maxIterations_);
		VectorValues xbar = system.x(ybar);
		return xbar;
	}

}
