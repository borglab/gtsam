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
	template<class Graph, class Config>
	SubgraphPCG<Graph, Config>::SubgraphPCG(const Graph& G, const Config& config) :
		maxIterations_(100), verbose_(false), epsilon_(1e-4), epsilon_abs_(1e-5) {

		// generate spanning tree and create ordering
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
		theta_bar_ = composePoses<Graph, Constraint, Pose, Config> (T_, tree, config[root]);

	}

	/* ************************************************************************* */
	template<class Graph, class Config>
	VectorConfig SubgraphPCG<Graph, Config>::linearizeAndOptimize(const Graph& g,
			const Config& theta_bar, const Ordering& ordering) const {

		VectorConfig zeros;
		BOOST_FOREACH(const Symbol& j, ordering) zeros.insert(j,zero(3));

		// build the subgraph PCG system
		GaussianFactorGraph Ab1 = T_.linearize(theta_bar);
		GaussianFactorGraph Ab2 = C_.linearize(theta_bar);
		const GaussianBayesNet Rc1 = Ab1.eliminate(ordering);
		VectorConfig xbar = gtsam::optimize(Rc1);
		SubgraphPreconditioner system(Rc1, Ab2, xbar);

		// Solve the subgraph PCG
		VectorConfig ybar = conjugateGradients<SubgraphPreconditioner, VectorConfig,
				Errors> (system, zeros, verbose_, epsilon_, epsilon_abs_, maxIterations_);
		VectorConfig xbar2 = system.x(ybar);
		return xbar2;
	}
}
