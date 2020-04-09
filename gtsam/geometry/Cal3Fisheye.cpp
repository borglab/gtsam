/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Fisheye.cpp
 * @date Apr 8, 2020
 * @author ghaggin
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3Fisheye.h>

namespace gtsam {

/* ************************************************************************* */
void Cal3Fisheye::print(const std::string& s_) const {
  Base::print(s_);
}

/* ************************************************************************* */
bool Cal3Fisheye::equals(const Cal3Fisheye& K, double tol) const {
  if (std::abs(fx_ - K.fx_) > tol || std::abs(fy_ - K.fy_) > tol || std::abs(s_ - K.s_) > tol ||
      std::abs(u0_ - K.u0_) > tol || std::abs(v0_ - K.v0_) > tol || std::abs(k1_ - K.k1_) > tol ||
      std::abs(k2_ - K.k2_) > tol || std::abs(k3_ - K.k3_) > tol || std::abs(k4_ - K.k4_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Cal3Fisheye Cal3Fisheye::retract(const Vector& d) const {
  return Cal3Fisheye(vector() + d);
}

/* ************************************************************************* */
Vector Cal3Fisheye::localCoordinates(const Cal3Fisheye& T2) const {
  return T2.vector() - vector();
}

}
/* ************************************************************************* */


