/**
 * @file summarization.h
 *
 * @brief Types and utility functions for summarization
 * 
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

// Sequential graph summarization
typedef FactorGraph<JacobianFactor> JacobianGraph;
typedef boost::shared_ptr<JacobianGraph> shared_jacobianGraph;

/**
 * Summarization function that eliminates a set of variables (does not convert to Jacobians)
 * NOTE: uses sequential solver - requires fully constrained system
 */
GaussianFactorGraph::shared_ptr summarizeGraphSequential(
    const GaussianFactorGraph& full_graph, const std::vector<Index>& indices, bool useQR = false);

/** Summarization that also converts keys to indices */
GaussianFactorGraph::shared_ptr summarizeGraphSequential(
    const GaussianFactorGraph& full_graph, const Ordering& ordering,
    const KeySet& saved_keys, bool useQR = false);

/**
 * Summarization function to remove a subset of variables from a system using
 * a partial cholesky approach.  This does not require that the system be fully constrained.
 *
 * Performs linearization to apply an ordering
 */
std::pair<GaussianFactorGraph,Ordering>
partialCholeskySummarization(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& overlap_keys);


} // \namespace gtsam


