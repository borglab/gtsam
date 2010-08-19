/*
 * GaussianJunctionTree.h
 * Created on: Jul 12, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: the Gaussian junction tree
 */

#pragma once

#include <gtsam/inference/JunctionTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

	/* ************************************************************************* */
	/**
	 * GaussianJunctionTree that does the optimization
	 */
	class GaussianJunctionTree: public JunctionTree<GaussianFactorGraph> {
	public:
		typedef JunctionTree<GaussianFactorGraph> Base;
		typedef Base::sharedClique sharedClique;

	protected:
		// back-substitute in topological sort order (parents first)
		void btreeBackSubstitue(BayesTree<GaussianConditional>::sharedClique current, VectorConfig& config);

	public :

		GaussianJunctionTree() : Base() {}

		// constructor
		GaussianJunctionTree(GaussianFactorGraph& fg, const Ordering& ordering) : Base(fg, ordering) {}

		// optimize the linear graph
		VectorConfig optimize();
	}; // GaussianJunctionTree

} // namespace gtsam
