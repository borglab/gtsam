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
 * @date    Dec 8, 2010
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/dllexport.h>

#include <boost/optional.hpp>

namespace gtsam {

class GaussianFactorGraph;
class Ordering;

/**
 * One SlotEntry stores the slot index for a variable, as well its dimension.
 */
struct GTSAM_EXPORT SlotEntry {
  size_t slot, dimension;
  SlotEntry(size_t _slot, size_t _dimension)
      : slot(_slot), dimension(_dimension) {}
  std::string toString() const;
};

/**
 * Scatter is an intermediate data structure used when building a HessianFactor
 * incrementally, to get the keys in the right order. The "scatter" is a map
 * from
 * global variable indices to slot indices in the union of involved variables.
 * We also include the dimensionality of the variable.
 */
class Scatter : public FastMap<Key, SlotEntry> {
 public:
  Scatter(const GaussianFactorGraph& gfg,
          boost::optional<const Ordering&> ordering = boost::none);
};

}  // \ namespace gtsam
