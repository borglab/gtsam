/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianGlobalConstraintFactor.h
 * @author  Richard Roberts
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @date    Dec 8, 2010
 */
#pragma once

#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/**
 * A Jacobian factor that is constrained on all or most of its keys
 *
 * Note that this isn't actually implemented any differently than a normal
 * Jacobian factor; it's just a different type to basically just act as a flag.
 */
class GTSAM_EXPORT JacobianGlobalConstraintFactor : public JacobianFactor {
 public:
  using JacobianFactor::JacobianFactor;
};  // JacobianGlobalConstraintFactor
    /// traits
template <>
struct traits<JacobianGlobalConstraintFactor>
    : public Testable<JacobianGlobalConstraintFactor> {};
}  // namespace gtsam

BOOST_CLASS_VERSION(gtsam::JacobianGlobalConstraintFactor, 1)
