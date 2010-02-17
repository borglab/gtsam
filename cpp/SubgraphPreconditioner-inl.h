/*
 * SubgraphPreconditioner-inl.h
 *
 *   Created on: Jan 17, 2010
 *       Author: nikai
 *  Description: subgraph preconditioning conjugate gradient solver
 */

#pragma once

#include <boost/tuple/tuple.hpp>
#include "SubgraphPreconditioner.h"

#include "graph-inl.h"
#include "iterative-inl.h"
#include "FactorGraph-inl.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class G, class T>
	SubgraphPCG<G, T>::SubgraphPCG(const G& g, const T& theta0) {
		initialize(g,theta0);
	}

	/* ************************************************************************* */
	template<class G, class T>
	void SubgraphPCG<G, T>::initialize(const G& g, const T& theta0) {

		// generate spanning tree
		PredecessorMap<Key> tree = g.template findMinimumSpanningTree<Key, Constraint>();
		list<Key> keys = predecessorMap2Keys(tree);

		// split the graph
		if (verbose_) cout << "generating spanning tree and split the graph ...";
		g.template split<Key, Constraint>(tree, T_, C_);
		if (verbose_) cout << T_.size() << " and " << C_.size() << " factors" << endl;

		// make the ordering
		list<Symbol> symbols;
		symbols.resize(keys.size());
		std::transform(keys.begin(), keys.end(), symbols.begin(), key2symbol<Key>);
		ordering_ = boost::shared_ptr<Ordering>(new Ordering(symbols));

		// compose the approximate solution
		Key root = keys.back();
		theta_bar_ = composePoses<G, Constraint, Pose, T> (T_, tree, theta0[root]);
	}

	/* ************************************************************************* */
	template<class G, class T>
	boost::shared_ptr<SubgraphPreconditioner> SubgraphPCG<G, T>::linearize(const G& g, const T& theta_bar) const {
		SubgraphPreconditioner::sharedFG Ab1 = T_.linearize_(theta_bar);
		SubgraphPreconditioner::sharedFG Ab2 = C_.linearize_(theta_bar);
#ifdef TIMING
		SubgraphPreconditioner::sharedBayesNet Rc1;
		SubgraphPreconditioner::sharedConfig xbar;
#else
		GaussianFactorGraph sacrificialAb1 = T_.linearize(theta_bar); // duplicate !!!!!
		SubgraphPreconditioner::sharedBayesNet Rc1 = sacrificialAb1.eliminate_(*ordering_);
		SubgraphPreconditioner::sharedConfig xbar = gtsam::optimize_(*Rc1);
#endif
		return boost::shared_ptr<SubgraphPreconditioner>(new SubgraphPreconditioner(Ab1, Ab2, Rc1, xbar));
	}

	/* ************************************************************************* */
	template<class G, class T>
	VectorConfig SubgraphPCG<G, T>::optimize(SubgraphPreconditioner& system) const {
		VectorConfig zeros = system.zero();

		// Solve the subgraph PCG
		VectorConfig ybar = conjugateGradients<SubgraphPreconditioner, VectorConfig,
				Errors> (system, zeros, verbose_, epsilon_, epsilon_abs_, maxIterations_);
		VectorConfig xbar = system.x(ybar);
		return xbar;
	}

}
