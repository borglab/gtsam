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
#include <gtsam/linear/GaussianFactorGraphOrdered.h>
#include <gtsam/linear/GaussianBayesTreeOrdered.h>
#include <gtsam/nonlinear/OrderingOrdered.h>

namespace gtsam {

/**
 * Backsubstitution-based conditioning - reduces conditionals to
 * densities on a sub-set of variables.
 *
 * Corner cases:
 *  - If no frontal vars are saved, returns a null pointer
 *
 *  @param initConditional the conditional from which to remove a variable
 *  @param saved_indices is the set of indices that should appear in the result
 *  @param solution is a full solution for the system
 */
GTSAM_UNSTABLE_EXPORT gtsam::GaussianConditionalOrdered::shared_ptr conditionDensity(const gtsam::GaussianConditionalOrdered::shared_ptr& initConditional,
    const std::set<gtsam::Index>& saved_indices, const gtsam::VectorValuesOrdered& solution);

/**
 * Backsubstitution-based conditioning for a complete Bayes Tree - reduces
 * conditionals by solving out variables to eliminate. Traverses the tree to
 * add the correct dummy factors whenever a separator is eliminated.
 */
GTSAM_UNSTABLE_EXPORT gtsam::GaussianFactorGraphOrdered conditionDensity(const gtsam::GaussianBayesTreeOrdered& bayesTree,
    const std::set<gtsam::Index>& saved_indices);


} // \namespace gtsam


