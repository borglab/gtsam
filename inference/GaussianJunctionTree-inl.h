/*
 * GaussianJunctionTree-inl.h
 *
 *   Created on: Jul 12, 2010
 *       Author: nikai
 *  Description: the Gaussian junction tree
 */

#pragma once

#include <boost/foreach.hpp>

#include "JunctionTree-inl.h"
#include "GaussianJunctionTree.h"

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	/**
	 * GaussianJunctionTree
	 */
	template <class FG>
	void GaussianJunctionTree<FG>::btreeBackSubstitue(typename BayesTree<GaussianConditional>::sharedClique current, VectorConfig& config) {
		// solve the bayes net in the current node
		typename BayesNet<GaussianConditional>::const_reverse_iterator it = current->rbegin();
		for (; it!=current->rend(); it++) {
			Vector x = (*it)->solve(config); // Solve for that variable
			config.insert((*it)->key(),x);   // store result in partial solution
		}

		// solve the bayes nets in the child nodes
		typedef typename BayesTree<GaussianConditional>::sharedClique sharedBayesClique;
		BOOST_FOREACH(sharedBayesClique child, current->children_) {
			btreeBackSubstitue(child, config);
		}
	}

	/* ************************************************************************* */
	template <class FG>
	VectorConfig GaussianJunctionTree<FG>::optimize() {
		// eliminate from leaves to the root
		typedef JunctionTree<FG> Base;
		BayesTree<GaussianConditional> bayesTree;
				this->eliminate<GaussianConditional>();

		// back-substitution
		VectorConfig result;
		btreeBackSubstitue(bayesTree.root(), result);
		return result;
	}


} //namespace gtsam
