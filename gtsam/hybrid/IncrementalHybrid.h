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
#include <gtsam/hybrid/HybridFactorGraph.h>

namespace gtsam {

class IncrementalHybrid {

 public:

  HybridBayesNet::shared_ptr hybridBayesNet_;
  HybridFactorGraph::shared_ptr remainingFactorGraph_;

  /**
   * Given new factors, perform an incremental update.
   * @param graph The new factors, should be linear only
   */
  void update(HybridFactorGraph graph, const Ordering &ordering);

};

};