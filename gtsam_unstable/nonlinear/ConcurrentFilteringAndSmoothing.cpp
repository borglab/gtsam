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

#include "ConcurrentFilteringAndSmoothing.h"

namespace gtsam {

void synchronize(ConcurrentFilter& filter, ConcurrentSmoother& smoother) {

  NonlinearFactorGraph smootherFactors, filterSumarization, smootherSummarization;
  Values smootherValues, rootValues;

  // Call the pre-sync functions of the filter and smoother
  filter.presync();
  smoother.presync();

  // Get the updates from the smoother and apply them to the filter
  smoother.getSummarizedFactors(smootherSummarization);
  filter.synchronize(smootherSummarization);

  // Get the updates from the filter and apply them to the smoother
  filter.getSmootherFactors(smootherFactors, smootherValues);
  filter.getSummarizedFactors(filterSumarization, rootValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, rootValues);

  // Call the post-sync functions of the filter and smoother
  filter.postsync();
  smoother.postsync();
}

}/// namespace gtsam
