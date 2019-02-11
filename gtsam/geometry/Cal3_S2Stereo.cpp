/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2Stereo.cpp
 * @brief  The most common 5DOF 3D->2D calibration + Stereo baseline
 * @author Chris Beall
 */

#include <gtsam/geometry/Cal3_S2Stereo.h>

#include <iostream>

namespace gtsam {
using namespace std;

/* ************************************************************************* */
void Cal3_S2Stereo::print(const std::string& s) const {
     K_.print(s+"K: ");
     std::cout << s << "Baseline: " << b_ << std::endl;
   }

/* ************************************************************************* */
bool Cal3_S2Stereo::equals(const Cal3_S2Stereo& other, double tol) const {
  if (fabs(b_ - other.b_) > tol) return false;
  return K_.equals(other.K_,tol);
}

/* ************************************************************************* */

} // namespace gtsam
