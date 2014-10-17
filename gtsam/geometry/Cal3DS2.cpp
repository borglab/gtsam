/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.cpp
 * @date Feb 28, 2010
 * @author ydjian
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3DS2.h>

namespace gtsam {

/* ************************************************************************* */
void Cal3DS2::print(const std::string& s_) const {
  Base::print(s_);
}

/* ************************************************************************* */
bool Cal3DS2::equals(const Cal3DS2& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol || fabs(s_ - K.s_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol || fabs(k1_ - K.k1_) > tol ||
      fabs(k2_ - K.k2_) > tol || fabs(p1_ - K.p1_) > tol || fabs(p2_ - K.p2_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Cal3DS2 Cal3DS2::retract(const Vector& d) const {
  return Cal3DS2(vector() + d);
}

/* ************************************************************************* */
Vector Cal3DS2::localCoordinates(const Cal3DS2& T2) const {
  return T2.vector() - vector();
}

}
/* ************************************************************************* */


