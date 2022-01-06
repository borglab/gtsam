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
        // Efficiently insert all the discrete keys to the final result.
        result.append(factor->discreteKeys());
      }
    }

    return result;
  }
};

}  // namespace gtsam
