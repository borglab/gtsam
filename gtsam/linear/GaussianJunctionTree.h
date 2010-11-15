/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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

#include <boost/shared_ptr.hpp>

namespace gtsam {

	/* ************************************************************************* */
	/**
	 * GaussianJunctionTree that does the optimization
	 */
	class GaussianJunctionTree: public JunctionTree<GaussianFactorGraph> {
	public:
	  typedef boost::shared_ptr<GaussianJunctionTree> shared_ptr;
		typedef JunctionTree<GaussianFactorGraph> Base;
		typedef Base::sharedClique sharedClique;

	protected:
		// back-substitute in topological sort order (parents first)
		void btreeBackSubstitute(const boost::shared_ptr<const BayesTree::Clique>& current, VectorValues& config) const;

	public :

		/** Default constructor */
		GaussianJunctionTree() : Base() {}

		/** Constructor from a factor graph.  Builds a VariableIndex. */
		GaussianJunctionTree(const GaussianFactorGraph& fg) : Base(fg) {}

    /** Construct from a factor graph and a pre-computed variable index. */
    GaussianJunctionTree(const GaussianFactorGraph& fg, const VariableIndex& variableIndex) : Base(fg, variableIndex) {}

		// optimize the linear graph
		VectorValues optimize() const;
	}; // GaussianJunctionTree

} // namespace gtsam
