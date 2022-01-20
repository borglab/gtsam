/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IncrementalHybrid.cpp
 * @brief   An incremental solver for hybrid factor graphs
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/IncrementalHybrid.h>

void gtsam::IncrementalHybrid::update(gtsam::HybridFactorGraph graph,
                                      const gtsam::Ordering &ordering) {
  if (hybridBayesNet_) {
    std::set<Key> allVars(ordering.begin(), ordering.end());
    for (auto &density : *hybridBayesNet_) {
      for (auto &key : density->frontals()) {
        if (allVars.find(key) != allVars.end()) {
          graph.push_back(density);
          break;
        }
      }
    }
  }

  // Eliminate partially.
  std::tie(hybridBayesNet_, remainingFactorGraph_) =
      graph.eliminatePartialSequential(ordering);
}
