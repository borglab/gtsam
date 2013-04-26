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
 * Summarization function to remove a subset of variables from a system with the
 * sequential solver. This does not require that the system be fully constrained.
 *
 * @param graph A full nonlinear graph
 * @param values The chosen linearization point
 * @param saved_keys is the set of keys for variables that should remain
 * @return a pair of the remaining graph and the ordering used for linearization
 */
std::pair<GaussianFactorGraph,Ordering> GTSAM_UNSTABLE_EXPORT
sequentialSummarization(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys);

/**
 * Summarization function to remove a subset of variables from a system using
 * a partial cholesky approach.  This does not require that the system be fully constrained.
 * Performs linearization to apply an ordering.
 *
 * @param graph A full nonlinear graph
 * @param values The chosen linearization point
 * @param saved_keys is the set of keys for variables that should remain
 * * @return a pair of the remaining graph and the ordering used for linearization
 */
std::pair<GaussianFactorGraph,Ordering> GTSAM_UNSTABLE_EXPORT
partialCholeskySummarization(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys);

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


