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

#include <gtsam/linear/GaussianFactorGraphOrdered.h>
#include <gtsam/nonlinear/OrderingOrdered.h>

namespace gtsam {

/**
 * Summarization function that eliminates a set of variables (does not convert to Jacobians)
 * NOTE: uses sequential solver - requires fully constrained system
 */
GaussianFactorGraphOrdered::shared_ptr GTSAM_UNSTABLE_EXPORT summarizeGraphSequential(
    const GaussianFactorGraphOrdered& full_graph, const std::vector<Index>& indices, bool useQR = false);

/** Summarization that also converts keys to indices */
GaussianFactorGraphOrdered::shared_ptr GTSAM_UNSTABLE_EXPORT summarizeGraphSequential(
    const GaussianFactorGraphOrdered& full_graph, const OrderingOrdered& ordering,
    const KeySet& saved_keys, bool useQR = false);

} // \namespace gtsam


