/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DCFactorGraph.h
 * @brief  Simple class for factor graphs of DCFactor type
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */
#pragma once

#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/inference/FactorGraph.h>

#include <algorithm>

namespace gtsam {

/**
 * Very simple class to create a factor graph with factors of type DCFactor
 */
class DCFactorGraph : public gtsam::FactorGraph<DCFactor> {
 public:
  using shared_ptr = boost::shared_ptr<DCFactorGraph>;

  DCFactorGraph() : FactorGraph<DCFactor>() {}

  /// Get all the discrete keys in all the factors in the DCFactorGraph
  DiscreteKeys discreteKeys() const {
    DiscreteKeys result;
    for (const sharedFactor& factor : *this) {
      if (factor) {
        // Insert all the discrete keys to the final result.
        for (auto&& key : factor->discreteKeys()) {
          // Check if key doesn't already exist. If it doesn't then add it.
          if (std::find(result.begin(), result.end(), key) == result.end()) {
            result.push_back(key);
          }
        }
      }
    }

    return result;
  }
};

}  // namespace gtsam
