/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentFilteringAndSmoothing.cpp
 * @brief   Base classes for the 'filter' and 'smoother' portion of the Concurrent
 *          Filtering and Smoothing architecture, as well as an external synchronization
 *          function. These classes act as an interface only.
 * @author  Stephen Williams
 */

// \callgraph

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

/* ************************************************************************* */
void synchronize(ConcurrentFilter& filter, ConcurrentSmoother& smoother) {

  NonlinearFactorGraph smootherFactors, filterSumarization, smootherSummarization;
  Values smootherValues, filterSeparatorValues, smootherSeparatorValues;

  // Call the pre-sync functions of the filter and smoother
  filter.presync();
  smoother.presync();

  // Get the updates from the smoother and apply them to the filter
  smoother.getSummarizedFactors(smootherSummarization, smootherSeparatorValues);
  filter.synchronize(smootherSummarization, smootherSeparatorValues);

  // Get the updates from the filter and apply them to the smoother
  filter.getSmootherFactors(smootherFactors, smootherValues);
  filter.getSummarizedFactors(filterSumarization, filterSeparatorValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, filterSeparatorValues);

  // Call the post-sync functions of the filter and smoother
  filter.postsync();
  smoother.postsync();
}

namespace internal {

/* ************************************************************************* */
NonlinearFactorGraph calculateMarginalFactors(const NonlinearFactorGraph& graph, const Values& theta,
    const KeySet& remainingKeys, const GaussianFactorGraph::Eliminate& eliminateFunction) {


  // Calculate the set of RootKeys = AllKeys \Intersect RemainingKeys
  KeySet rootKeys;
  KeySet allKeys(graph.keys());
  std::set_intersection(allKeys.begin(), allKeys.end(), remainingKeys.begin(), remainingKeys.end(), std::inserter(rootKeys, rootKeys.end()));

  // Calculate the set of MarginalizeKeys = AllKeys - RemainingKeys
  KeySet marginalizeKeys;
  std::set_difference(allKeys.begin(), allKeys.end(), remainingKeys.begin(), remainingKeys.end(), std::inserter(marginalizeKeys, marginalizeKeys.end()));

  if(marginalizeKeys.size() == 0) {
    // There are no keys to marginalize. Simply return the input factors
    return graph;
  } else {
    // Create the linear factor graph
    GaussianFactorGraph linearFactorGraph = *graph.linearize(theta);
    // .first is the eliminated Bayes tree, while .second is the remaining factor graph
    GaussianFactorGraph marginalLinearFactors = *linearFactorGraph.eliminatePartialMultifrontal(
        KeyVector(marginalizeKeys.begin(), marginalizeKeys.end()), eliminateFunction).second;

    // Wrap in nonlinear container factors
    NonlinearFactorGraph marginalFactors;
    marginalFactors.reserve(marginalLinearFactors.size());
    for(const GaussianFactor::shared_ptr& gaussianFactor: marginalLinearFactors) {
      marginalFactors.emplace_shared<LinearContainerFactor>(gaussianFactor, theta);
    }

    return marginalFactors;
  }
}

/* ************************************************************************* */
}/// namespace internal

/* ************************************************************************* */
}/// namespace gtsam
