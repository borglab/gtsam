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
	SubgraphPCG<G, T>::SubgraphPCG(const G& g, const T& theta0) :
		maxIterations_(100), verbose_(false), epsilon_(1e-4), epsilon_abs_(1e-5) {

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
	SubgraphPreconditioner SubgraphPCG<G, T>::linearize(const G& g, const T& theta_bar) const {
		GaussianFactorGraph Ab1 = T_.linearize(theta_bar);
		SubgraphPreconditioner::sharedFG Ab2 = C_.linearize_(theta_bar);
		SubgraphPreconditioner::sharedBayesNet Rc1 = Ab1.eliminate_(*ordering_);
		SubgraphPreconditioner::sharedConfig xbar = gtsam::optimize_(*Rc1);
		return SubgraphPreconditioner(Rc1, Ab2, xbar);
	}

	/* ************************************************************************* */
	template<class G, class T>
	VectorConfig SubgraphPCG<G, T>::optimize(SubgraphPreconditioner& system) const {
		//TODO: 3 is hard coded here
		VectorConfig zeros;
		BOOST_FOREACH(const Symbol& j, *ordering_) zeros.insert(j,zero(Pose::dim()));

		// Solve the subgraph PCG
		VectorConfig ybar = conjugateGradients<SubgraphPreconditioner, VectorConfig,
				Errors> (system, zeros, verbose_, epsilon_, epsilon_abs_, maxIterations_);
		VectorConfig xbar = system.x(ybar);
		return xbar;
	}

}
