/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3.cpp
 * @brief  Common code for all calibration models.
 * @author Frank Dellaert
 */

#include <gtsam/geometry/Cal3.h>

#include <cmath>
#include <fstream>
#include <iostream>

namespace gtsam {

/* ************************************************************************* */
Cal3::Cal3(double fov, int w, int h)
    : s_(0), u0_((double)w / 2.0), v0_((double)h / 2.0) {
  double a = fov * M_PI / 360.0;  // fov/2 in radians
  // old formula: fx_ = (double) w * tan(a);
  fx_ = double(w) / (2.0 * tan(a));
  fy_ = fx_;
}

/* ************************************************************************* */
Cal3::Cal3(const std::string& path)
    : fx_(320), fy_(320), s_(0), u0_(320), v0_(140) {
  const auto buffer = path + std::string("/calibration_info.txt");
  std::ifstream infile(buffer, std::ios::in);

  if (infile) {
    infile >> fx_ >> fy_ >> s_ >> u0_ >> v0_;
  } else {
    throw std::runtime_error("Cal3: Unable to load the calibration");
  }

  infile.close();
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Cal3& cal) {
  os << "fx: " << cal.fx() << ", fy: " << cal.fy() << ", s: " << cal.skew()
     << ", px: " << cal.px() << ", py: " << cal.py();
  return os;
}

/* ************************************************************************* */
void Cal3::print(const std::string& s) const { gtsam::print((Matrix)K(), s); }

/* ************************************************************************* */
bool Cal3::equals(const Cal3& K, double tol) const {
  return (std::fabs(fx_ - K.fx_) < tol && std::fabs(fy_ - K.fy_) < tol &&
          std::fabs(s_ - K.s_) < tol && std::fabs(u0_ - K.u0_) < tol &&
          std::fabs(v0_ - K.v0_) < tol);
}

/* ************************************************************************* */

}  // \ namespace gtsam
