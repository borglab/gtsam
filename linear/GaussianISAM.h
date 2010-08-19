/**
 * @file    GaussianISAM
 * @brief   Linear ISAM only
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <gtsam/inference/ISAM.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

	typedef ISAM<GaussianConditional> GaussianISAM;

	// recursively optimize this conditional and all subtrees
	void optimize(const GaussianISAM::sharedClique& clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize(const GaussianISAM& bayesTree);

}/// namespace gtsam
