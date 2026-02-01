/**
 * @file BearingS2.cpp
 *
 * @date Jan 26, 2012
 * @author Alex Cunningham
 */

#include <iostream>

#include <gtsam_unstable/geometry/BearingS2.h>

#include <cassert>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
void BearingS2::print(const std::string& s) const {
  cout << s << " azimuth: " << azimuth_.theta() << " elevation: " << elevation_.theta() << endl;
}

/* ************************************************************************* */
bool BearingS2::equals(const BearingS2& x, double tol) const {
  return azimuth_.equals(x.azimuth_, tol) && elevation_.equals(x.elevation_, tol);
}

/* ************************************************************************* */
BearingS2 BearingS2::fromDownwardsObservation(const Pose3& A, const Point3& B) {
  //  Cnb = DCMnb(Att);
  Matrix Cnb = A.rotation().matrix().transpose();

  //  Cbc = [0,0,1;0,1,0;-1,0,0];
  Matrix Cbc = (Matrix(3,3) <<
      0.,0.,1.,
      0.,1.,0.,
      -1.,0.,0.).finished();
  //  p_rel_c = Cbc*Cnb*(PosObj - Pos);
  Vector p_rel_c = Cbc*Cnb*(B - A.translation());

  // FIXME: the matlab code checks for p_rel_c(0) greater than

  //  azi = atan2(p_rel_c(2),p_rel_c(1));
  double azimuth = atan2(p_rel_c(1),p_rel_c(0));
  //  elev = atan2(p_rel_c(3),sqrt(p_rel_c(1)^2 + p_rel_c(2)^2));
  double elevation = atan2(p_rel_c(2),sqrt(p_rel_c(0) * p_rel_c(0) + p_rel_c(1) * p_rel_c(1)));
  return BearingS2(azimuth, elevation);
}

/* ************************************************************************* */
BearingS2 BearingS2::fromForwardObservation(const Pose3& A, const Point3& B) {
  //  Cnb = DCMnb(Att);
  Matrix Cnb = A.rotation().matrix().transpose();

  Vector p_rel_c = Cnb*(B - A.translation());

  // FIXME: the matlab code checks for p_rel_c(0) greater than

  //  azi = atan2(p_rel_c(2),p_rel_c(1));
  double azimuth = atan2(p_rel_c(1),p_rel_c(0));
  //  elev = atan2(p_rel_c(3),sqrt(p_rel_c(1)^2 + p_rel_c(2)^2));
  double elevation = atan2(p_rel_c(2),sqrt(p_rel_c(0) * p_rel_c(0) + p_rel_c(1) * p_rel_c(1)));
  return BearingS2(azimuth, elevation);
}

/* ************************************************************************* */
BearingS2 BearingS2::retract(const Vector& v) const {
  assert(v.size() == 2);
  return BearingS2(azimuth_.retract(v.head(1)), elevation_.retract(v.tail(1)));
}

/* ************************************************************************* */
Vector BearingS2::localCoordinates(const BearingS2& x) const {
  return (Vector(2) << azimuth_.localCoordinates(x.azimuth_)(0),
                    elevation_.localCoordinates(x.elevation_)(0)).finished();
}

} // \namespace gtsam
