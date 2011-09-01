/**
 * @file    GaussianISAM
 * @brief   Full non-linear ISAM.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/slam/simulated2D.h>
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/nonlinear/ISAM2-inl.h>

namespace gtsam {

/**
 * @ingroup ISAM2
 * @brief The main ISAM2 class that is exposed to gtsam users, see ISAM2 for usage.
 *
 * This is a thin wrapper around an ISAM2 class templated on
 * GaussianConditional, and the values on which that GaussianISAM2 is
 * templated.
 *
 * @tparam VALUES The LieValues or TupleValues\Emph{N} that contains the
 * variables.
 */
template <class VALUES>
class GaussianISAM2 : public ISAM2<GaussianConditional, VALUES> {
public:
  /** Create an empty ISAM2 instance */
  GaussianISAM2(const ISAM2Params& params) : ISAM2<GaussianConditional, VALUES>(params) {}

  /** Create an empty ISAM2 instance using the default set of parameters (see ISAM2Params) */
  GaussianISAM2() : ISAM2<GaussianConditional, VALUES>() {}
};

// optimize the BayesTree, starting from the root
void optimize2(const BayesTree<GaussianConditional>::sharedClique& root, VectorValues& delta);

// optimize the BayesTree, starting from the root; "replaced" needs to contain
// all variables that are contained in the top of the Bayes tree that has been
// redone; "delta" is the current solution, an offset from the linearization
// point; "threshold" is the maximum change against the PREVIOUS delta for
// non-replaced variables that can be ignored, ie. the old delta entry is kept
// and recursive backsubstitution might eventually stop if none of the changed
// variables are contained in the subtree.
// returns the number of variables that were solved for
int optimize2(const BayesTree<GaussianConditional>::sharedClique& root,
    double threshold, const std::vector<bool>& replaced, Permuted<VectorValues>& delta);

// calculate the number of non-zero entries for the tree starting at clique (use root for complete matrix)
int calculate_nnz(const BayesTree<GaussianConditional>::sharedClique& clique);

}/// namespace gtsam
