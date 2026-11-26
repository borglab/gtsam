/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

namespace gtsam {

// MATLAB proxy for the native set type. Python uses its STL set caster.
#include <gtsam/nonlinear/WnoaStateData.h>
class StateDataSet {
  StateDataSet();
  size_t size() const;
  void insert(const gtsam::StateData& state);
};

}  // namespace gtsam
