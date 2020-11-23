/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Scatter.h
 * @brief   Maps global variable indices to slot indices
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @date    June 2015
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/dllexport.h>

namespace gtsam {

class GaussianFactorGraph;
class Ordering;

/// One SlotEntry stores the slot index for a variable, as well its dim.
struct GTSAM_EXPORT SlotEntry {
  Key key;
  size_t dimension;
  SlotEntry(Key _key, size_t _dimension) : key(_key), dimension(_dimension) {}
  std::string toString() const;
  friend bool operator<(const SlotEntry& p, const SlotEntry& q) {
    return p.key < q.key;
  }
  static bool Zero(const SlotEntry& p) { return p.dimension==0;}
};

/**
 * Scatter is an intermediate data structure used when building a HessianFactor
 * incrementally, to get the keys in the right order. In spirit, it is a map
 * from global variable indices to slot indices in the union of involved
 * variables. We also include the dimensionality of the variable.
 */
class Scatter : public FastVector<SlotEntry> {
 public:
  /// Default Constructor
   GTSAM_EXPORT Scatter() {}

  /// Construct from gaussian factor graph, without ordering
   GTSAM_EXPORT explicit Scatter(const GaussianFactorGraph& gfg);

  /// Construct from gaussian factor graph, with (partial or complete) ordering
   GTSAM_EXPORT explicit Scatter(const GaussianFactorGraph& gfg, const Ordering& ordering);

  /// Add a key/dim pair
   GTSAM_EXPORT void add(Key key, size_t dim);

 private:
  /// Find the SlotEntry with the right key (linear time worst case)
  iterator find(Key key);
};

}  // \ namespace gtsam
