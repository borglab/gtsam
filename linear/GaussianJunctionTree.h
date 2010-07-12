/*
 * GaussianJunctionTree.h
 *
 *   Created on: Jul 12, 2010
 *       Author: nikai
 *  Description: the Gaussian junction tree
 */

#pragma once

#include "JunctionTree.h"
#include "GaussianConditional.h"
#include "GaussianFactorGraph.h"

namespace gtsam {

	/* ************************************************************************* */
	/**
	 * GaussianJunctionTree that does the optimization
	 */
	template <class FG>
	class GaussianJunctionTree: public JunctionTree<FG> {
	public:
		typedef JunctionTree<FG> Base;
		typedef typename JunctionTree<FG>::sharedClique sharedClique;

	protected:
		// back-substitute in topological sort order (parents first)
		void btreeBackSubstitue(typename BayesTree<GaussianConditional>::sharedClique current, VectorConfig& config);

	public :

		GaussianJunctionTree() : Base() {}

		// constructor
		GaussianJunctionTree(FG& fg, const Ordering& ordering) : Base(fg, ordering) {}

		// optimize the linear graph
		VectorConfig optimize();
	}; // GaussianJunctionTree

} // namespace gtsam
