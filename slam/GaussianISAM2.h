/**
 * @file    GaussianISAM
 * @brief   Full non-linear ISAM.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <gtsam/inference/ISAM2.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/slam/simulated2D.h>
#include <gtsam/slam/planarSLAM.h>

namespace gtsam {

	typedef ISAM2<GaussianConditional, simulated2D::Config> GaussianISAM2;

	// recursively optimize this conditional and all subtrees
	void optimize2(const GaussianISAM2::sharedClique& clique, double threshold, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize2(const GaussianISAM2& bayesTree, double threshold = 0.);


	// todo: copy'n'paste to avoid template hell

	typedef ISAM2<GaussianConditional, planarSLAM::Config> GaussianISAM2_P;

	// recursively optimize this conditional and all subtrees
//	void optimize2(const GaussianISAM2_P::sharedClique& clique, VectorConfig& result);

	// optimize the BayesTree, starting from the root
	VectorConfig optimize2(const GaussianISAM2_P& bayesTree, double threshold = 0.);

	// calculate the number of non-zero entries for the tree starting at clique (use root for complete matrix)
	int calculate_nnz(const GaussianISAM2::sharedClique& clique);

}/// namespace gtsam
