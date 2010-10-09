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

#include <gtsam/inference/graph-inl.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Graph, class Values>
	SubgraphSolver<Graph, Values>::SubgraphSolver(const Graph& G, const Values& theta0) {
		initialize(G,theta0);
	}

	/* ************************************************************************* */
	template<class Graph, class Values>
	void SubgraphSolver<Graph, Values>::initialize(const Graph& G, const Values& theta0) {

		// generate spanning tree
		PredecessorMap<Key> tree = G.template findMinimumSpanningTree<Key, Constraint>();
		list<Key> keys = predecessorMap2Keys(tree);

		// split the graph
		if (verbose_) cout << "generating spanning tree and split the graph ...";
		G.template split<Key, Constraint>(tree, T_, C_);
		if (verbose_) cout << T_.size() << " and " << C_.size() << " factors" << endl;

		// make the ordering
		list<Symbol> symbols;
		symbols.resize(keys.size());
		std::transform(keys.begin(), keys.end(), symbols.begin(), key2symbol<Key>);
		ordering_ = boost::shared_ptr<Ordering>(new Ordering(symbols));

		// compose the approximate solution
		Key root = keys.back();
		theta_bar_ = composePoses<Graph, Constraint, Pose, Values> (T_, tree, theta0[root]);
	}

	/* ************************************************************************* */
	template<class Graph, class Values>
	boost::shared_ptr<SubgraphPreconditioner> SubgraphSolver<Graph, Values>::linearize(const Graph& G, const Values& theta_bar) const {
		SubgraphPreconditioner::sharedFG Ab1 = T_.linearize(theta_bar);
		SubgraphPreconditioner::sharedFG Ab2 = C_.linearize(theta_bar);
#ifdef TIMING
		SubgraphPreconditioner::sharedBayesNet Rc1;
		SubgraphPreconditioner::sharedValues xbar;
#else
		GaussianFactorGraph sacrificialAb1 = *Ab1; // duplicate !!!!!
		SubgraphPreconditioner::sharedBayesNet Rc1 = sacrificialAb1.eliminate_(*ordering_);
		SubgraphPreconditioner::sharedValues xbar = gtsam::optimize_(*Rc1);
#endif
		// TODO: there does not seem to be a good reason to have Ab1_
		// It seems only be used to provide an ordering for creating sparse matrices
		return boost::shared_ptr<SubgraphPreconditioner>(new SubgraphPreconditioner(Ab1, Ab2, Rc1, xbar));
	}

	/* ************************************************************************* */
	template<class Graph, class Values>
	VectorValues SubgraphSolver<Graph, Values>::optimize(SubgraphPreconditioner& system) const {
		VectorValues zeros = system.zero();

		// Solve the subgraph PCG
		VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues,
				Errors> (system, zeros, verbose_, epsilon_, epsilon_abs_, maxIterations_);
		VectorValues xbar = system.x(ybar);
		return xbar;
	}

}
