/**
 * @file    GaussianISAM
 * @brief   Full non-linear ISAM.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include "ISAM2.h"
#include "GaussianConditional.h"
#include "GaussianFactor.h"

namespace gtsam {

	typedef ISAM2<GaussianConditional, VectorConfig> GaussianISAM2;

	// recursively optimize this conditional and all subtrees
	void optimize2(const GaussianISAM2::sharedClique& clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize2(const GaussianISAM2& bayesTree);

}/// namespace gtsam
