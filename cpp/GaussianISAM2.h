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
#include "simulated2D.h"
#include "planarSLAM.h"

namespace gtsam {

	typedef ISAM2<GaussianConditional, simulated2D::Config> GaussianISAM2;

	// recursively optimize this conditional and all subtrees
	void optimize2(const GaussianISAM2::sharedClique& clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize2(const GaussianISAM2& bayesTree);


	// todo: copy'n'paste to avoid template hell

	typedef ISAM2<GaussianConditional, planarSLAM::Config> GaussianISAM2_P;

	// recursively optimize this conditional and all subtrees
//	void optimize2(const GaussianISAM2_P::sharedClique& clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize2(const GaussianISAM2_P& bayesTree);

}/// namespace gtsam
