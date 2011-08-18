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

	typedef ISAM2<GaussianConditional, simulated2D::Values> GaussianISAM2;
	typedef ISAM2<GaussianConditional, planarSLAM::Values> GaussianISAM2_P;

	// optimize the BayesTree, starting from the root
	void optimize2(const GaussianISAM2::sharedClique& root, VectorValues& delta);

	// optimize the BayesTree, starting from the root; "replaced" needs to contain
	// all variables that are contained in the top of the Bayes tree that has been
	// redone; "delta" is the current solution, an offset from the linearization
	// point; "threshold" is the maximum change against the PREVIOUS delta for
	// non-replaced variables that can be ignored, ie. the old delta entry is kept
	// and recursive backsubstitution might eventually stop if none of the changed
	// variables are contained in the subtree.
	// returns the number of variables that were solved for
	int optimize2(const GaussianISAM2::sharedClique& root,
			double threshold, const std::vector<bool>& replaced, Permuted<VectorValues>& delta);

	// calculate the number of non-zero entries for the tree starting at clique (use root for complete matrix)
	int calculate_nnz(const GaussianISAM2::sharedClique& clique);

}/// namespace gtsam
