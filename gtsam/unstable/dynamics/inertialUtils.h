/**
 * @file inertialUtils.h
 *
 * @brief Utility functions for working with dynamic systems - derived from Mitch's matlab code
 * This is mostly just syntactic sugar for working with dynamic systems that use
 * Euler angles to specify the orientation of a robot.
 *
 * @date Nov 28, 2011
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/geometry/Rot3.h>

namespace gtsam {
namespace dynamics {

/// RRTMbn - Function computes the rotation rate transformation matrix from
/// body axis rates to euler angle (global) rates
Matrix RRTMbn(const Vector& euler);

Matrix RRTMbn(const Rot3& att);

/// RRTMnb - Function computes the rotation rate transformation matrix from
/// euler angle rates to body axis rates
Matrix RRTMnb(const Vector& euler);

Matrix RRTMnb(const Rot3& att);

} // \namespace dynamics
} // \namespace gtsam
