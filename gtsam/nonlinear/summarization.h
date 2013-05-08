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
 * @param useQR uses QR as the elimination algorithm if true, Cholesky otherwise
 * @return a pair of the remaining graph and the ordering used for linearization
 */
std::pair<GaussianFactorGraph,Ordering> GTSAM_EXPORT
summarize(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys, bool useQR = true);

/**
 * Performs the same summarization technique used in summarize(), but returns the
 * result as a NonlinearFactorGraph comprised of LinearContainerFactors.
 *
 * @param graph A full nonlinear graph
 * @param values The chosen linearization point
 * @param saved_keys is the set of keys for variables that should remain
 * @param useQR uses QR as the elimination algorithm if true, Cholesky otherwise
 * @return a NonlinearFactorGraph with linear factors
 */
NonlinearFactorGraph GTSAM_EXPORT summarizeAsNonlinearContainer(
    const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys, bool useQR = true);

} // \namespace gtsam


