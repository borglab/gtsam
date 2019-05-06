/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SOn.cpp
 * @brief   Definitions of dynamic specializations of SO(n)
 * @author  Frank Dellaert
 * @date    March 2019
 */

#include <gtsam/geometry/SOn.h>

namespace gtsam {

template <>
SOn LieGroup<SOn, Eigen::Dynamic>::compose(const SOn& g, DynamicJacobian H1,
                                           DynamicJacobian H2) const {
  if (H1) *H1 = g.inverse().AdjointMap();
  if (H2) *H2 = SOn::IdentityJacobian(g.rows());
  return derived() * g;
}

template <>
SOn LieGroup<SOn, Eigen::Dynamic>::between(const SOn& g, DynamicJacobian H1,
                                           DynamicJacobian H2) const {
  SOn result = derived().inverse() * g;
  if (H1) *H1 = -result.inverse().AdjointMap();
  if (H2) *H2 = SOn::IdentityJacobian(g.rows());
  return result;
}

}  // namespace gtsam
