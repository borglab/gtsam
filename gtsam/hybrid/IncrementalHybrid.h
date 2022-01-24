/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IncrementalHybrid.h
 * @brief   An incremental solver for hybrid factor graphs
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/GaussianHybridFactorGraph.h>

namespace gtsam {

class IncrementalHybrid {

 public:

  HybridBayesNet::shared_ptr hybridBayesNet_;
  GaussianHybridFactorGraph::shared_ptr remainingFactorGraph_;

  /**
   * Given new factors, perform an incremental update.
   * The relevant densities in the `hybridBayesNet` will be added to the input
   * graph (fragment), and then eliminated according to the `ordering` presented.
   *
   * @param graph The new factors, should be linear only
   * @param ordering The ordering for elimination
   */
  void update(GaussianHybridFactorGraph graph, const Ordering &ordering);

};

};
