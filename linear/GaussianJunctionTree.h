/**
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
		void btreeBackSubstitute(const boost::shared_ptr<const BayesTree::Clique>& current, VectorConfig& config) const;

	public :

		GaussianJunctionTree() : Base() {}

		// constructor
		GaussianJunctionTree(const GaussianFactorGraph& fg) : Base(fg) {}

		// optimize the linear graph
		VectorConfig optimize() const;
	}; // GaussianJunctionTree

} // namespace gtsam
