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
 * @author Varun Agrawal
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3DS2& cal) {
  os << (Cal3DS2_Base&)cal;
  return os;
}

/* ************************************************************************* */
void Cal3DS2::print(const std::string& s_) const { Base::print(s_); }

/* ************************************************************************* */
bool Cal3DS2::equals(const Cal3DS2& K, double tol) const {
  const Cal3DS2_Base* base = dynamic_cast<const Cal3DS2_Base*>(&K);
  return Cal3DS2_Base::equals(*base, tol);
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
