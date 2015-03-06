/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for GPS factor
 *  @date   January 28, 2014
 **/

#include "GPSFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void GPSFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "GPSFactor on " << keyFormatter(key()) << "\n";
  nZ_gps.print("  prior mean: ");
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && nZ_gps.equals(e->nZ_gps, tol);
}

//***************************************************************************
Vector GPSFactor::evaluateError(const Pose3& nTb,
    boost::optional<Matrix&> H) const {
  if (bTg_) {
    // do transform
    Matrix6 D_nTg_nTb;
    Pose3 nTg = nTb.compose(*bTg_, H ? &D_nTg_nTb : 0);
    Matrix36 D_nT_nTg;
    Point3 nT_g = nTg.translation(H ? &D_nT_nTg : 0);
    if (H)
      *H = D_nT_nTg * D_nTg_nTb;
    return (nT_g - nZ_gps).vector();
  } else {
    // by default, we assume bTg is identity
    const Pose3& nTg = nTb;
    return (nTg.translation(H) - nZ_gps).vector();
  }
}

//***************************************************************************
pair<Pose3, Vector3> GPSFactor::EstimateState(double t1, const Point3& NED1,
    double t2, const Point3& NED2, double timestamp) {
  // Estimate initial velocity as difference in NED frame
  double dt = t2 - t1;
  Point3 nV = (NED2 - NED1) / dt;

  // Estimate initial position as linear interpolation
  Point3 nT = NED1 + nV * (timestamp - t1);

  // Estimate Rotation
  double yaw = atan2(nV.y(), nV.x());
  Rot3 nRy = Rot3::yaw(yaw); // yaw frame
  Point3 yV = nRy.inverse() * nV; // velocity in yaw frame
  double pitch = -atan2(yV.z(), yV.x()), roll = 0;
  Rot3 nRb = Rot3::ypr(yaw, pitch, roll);

  // Construct initial pose
  Pose3 nTb(nRb, nT); // nTb

  return make_pair(nTb, nV.vector());
}
//***************************************************************************

}/// namespace gtsam
