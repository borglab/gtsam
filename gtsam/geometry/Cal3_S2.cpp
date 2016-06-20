/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2.cpp
 * @brief  The most common 5DOF 3D->2D calibration
 * @author Frank Dellaert
 */

#include <gtsam/geometry/Cal3_S2.h>

#include <cmath>
#include <fstream>
#include <iostream>

namespace gtsam {
using namespace std;

/* ************************************************************************* */
Cal3_S2::Cal3_S2(double fov, int w, int h) :
    s_(0), u0_((double) w / 2.0), v0_((double) h / 2.0) {
  double a = fov * M_PI / 360.0; // fov/2 in radians
  fx_ = (double) w / (2.0 * tan(a)); //    old formula: fx_ = (double) w * tan(a);
  fy_ = fx_;
}

/* ************************************************************************* */
Cal3_S2::Cal3_S2(const std::string &path) :
    fx_(320), fy_(320), s_(0), u0_(320), v0_(140) {

  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "%s/calibration_info.txt", path.c_str());
  std::ifstream infile(buffer, std::ios::in);

  if (infile)
    infile >> fx_ >> fy_ >> s_ >> u0_ >> v0_;
  else {
    printf("Unable to load the calibration\n");
    exit(0);
  }

  infile.close();
}

/* ************************************************************************* */
ostream& operator<<(ostream& os, const Cal3_S2& cal) {
  os << "{fx: " << cal.fx() << ", fy: " << cal.fy() << ", s:" << cal.skew() << ", px:" << cal.px()
     << ", py:" << cal.py() << "}";
  return os;
}

/* ************************************************************************* */
void Cal3_S2::print(const std::string& s) const {
  gtsam::print((Matrix)matrix(), s);
}

/* ************************************************************************* */
bool Cal3_S2::equals(const Cal3_S2& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol)
    return false;
  if (fabs(fy_ - K.fy_) > tol)
    return false;
  if (fabs(s_ - K.s_) > tol)
    return false;
  if (fabs(u0_ - K.u0_) > tol)
    return false;
  if (fabs(v0_ - K.v0_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Point2 Cal3_S2::uncalibrate(const Point2& p, OptionalJacobian<2, 5> Dcal,
    OptionalJacobian<2, 2> Dp) const {
  const double x = p.x(), y = p.y();
  if (Dcal)
    *Dcal << x, 0.0, y, 1.0, 0.0, 0.0, y, 0.0, 0.0, 1.0;
  if (Dp)
    *Dp << fx_, s_, 0.0, fy_;
  return Point2(fx_ * x + s_ * y + u0_, fy_ * y + v0_);
}

/* ************************************************************************* */
Point2 Cal3_S2::calibrate(const Point2& p, OptionalJacobian<2,5> Dcal,
                           OptionalJacobian<2,2> Dp) const {
    const double u = p.x(), v = p.y();
    double delta_u = u - u0_, delta_v = v - v0_;
    double inv_fx = 1/ fx_, inv_fy = 1/fy_;
    double inv_fy_delta_v = inv_fy * delta_v, inv_fx_s_inv_fy = inv_fx * s_ * inv_fy;
    Point2 point(inv_fx * (delta_u - s_ * inv_fy_delta_v),
                  inv_fy_delta_v);
    if(Dcal)
       *Dcal << - inv_fx * point.x(), inv_fx * s_ * inv_fy * inv_fy_delta_v,  -inv_fx * point.y(),
            -inv_fx,  inv_fx_s_inv_fy,
            0, -inv_fy * point.y(), 0,  0, -inv_fy;
    if(Dp)
        *Dp << inv_fx, -inv_fx_s_inv_fy, 0, inv_fy;
    return point;
}

/* ************************************************************************* */
Vector3 Cal3_S2::calibrate(const Vector3& p) const {
  return matrix_inverse() * p;
}

/* ************************************************************************* */

} // namespace gtsam
