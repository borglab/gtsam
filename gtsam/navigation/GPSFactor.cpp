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
  nT_.print("  prior mean: ");
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool GPSFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && nT_.equals(e->nT_, tol);
}

//***************************************************************************
Vector GPSFactor::evaluateError(const Pose3& p,
    boost::optional<Matrix&> H) const {
  if (H) {
    H->resize(3, 6);
    H->block < 3, 3 > (0, 0) << zeros(3, 3);
    H->block < 3, 3 > (0, 3) << p.rotation().matrix();
  }
  return (p.translation() -nT_).vector();
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
