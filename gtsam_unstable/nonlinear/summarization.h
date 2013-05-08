/**
 * @file summarization.h
 *
 * @brief Types and utility functions for summarization
 * 
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/base/dllexport.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

/**
 * Summarization function that eliminates a set of variables (does not convert to Jacobians)
 * NOTE: uses sequential solver - requires fully constrained system
 */
GaussianFactorGraph::shared_ptr GTSAM_UNSTABLE_EXPORT summarizeGraphSequential(
    const GaussianFactorGraph& full_graph, const std::vector<Index>& indices, bool useQR = false);

/** Summarization that also converts keys to indices */
GaussianFactorGraph::shared_ptr GTSAM_UNSTABLE_EXPORT summarizeGraphSequential(
    const GaussianFactorGraph& full_graph, const Ordering& ordering,
    const KeySet& saved_keys, bool useQR = false);

} // \namespace gtsam


