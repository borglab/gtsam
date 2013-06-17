/**
 * @file summarization.h
 *
 * @brief Types and utility functions for summarization
 * 
 * @date Jun 22, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

// Flag to control how summarization should be performed
typedef enum {
  PARTIAL_QR = 0,         /// Uses QR solver to eliminate, does not require fully constrained system
  PARTIAL_CHOLESKY = 1,   /// Uses Cholesky solver, does not require fully constrained system
  SEQUENTIAL_QR = 2,      /// Uses QR to compute full joint graph (needs fully constrained system)
  SEQUENTIAL_CHOLESKY = 3 /// Uses Cholesky to compute full joint graph (needs fully constrained system)
} SummarizationMode;

/**
 * Summarization function to remove a subset of variables from a system with the
 * sequential solver. This does not require that the system be fully constrained.
 *
 * Requirement: set of keys in the graph should match the set of keys in the
 * values structure.
 *
 * @param graph A full nonlinear graph
 * @param values The chosen linearization point
 * @param saved_keys is the set of keys for variables that should remain
 * @param mode controls what elimination technique and requirements to use
 * @return a pair of the remaining graph and the ordering used for linearization
 */
std::pair<GaussianFactorGraph,Ordering> GTSAM_EXPORT
summarize(const NonlinearFactorGraph& graph, const Values& values,
    const KeySet& saved_keys, SummarizationMode mode = PARTIAL_QR);

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
    const KeySet& saved_keys, SummarizationMode mode = PARTIAL_QR);

} // \namespace gtsam


