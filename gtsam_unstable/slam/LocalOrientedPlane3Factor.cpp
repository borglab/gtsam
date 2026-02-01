/*
 * LocalOrientedPlane3Factor.cpp
 *
 *  Author: David Wisth
 *  Created on: February 12, 2021
 */

#include "LocalOrientedPlane3Factor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void LocalOrientedPlane3Factor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << (s == "" ? "" : "\n");
  cout << "LocalOrientedPlane3Factor Factor (" << keyFormatter(key<1>()) << ", "
       << keyFormatter(key<2>()) << ", " << keyFormatter(key<3>()) << ")\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
Vector LocalOrientedPlane3Factor::evaluateError(const Pose3& wTwi,
    const Pose3& wTwa, const OrientedPlane3& a_plane,
    OptionalMatrixType H1, OptionalMatrixType H2,
    OptionalMatrixType H3) const {

  Matrix66 aTai_H_wTwa, aTai_H_wTwi;
  Matrix36 predicted_H_aTai;
  Matrix33 predicted_H_plane, error_H_predicted;

  // Find the relative transform from anchor to sensor frame.
  const Pose3 aTai = wTwa.transformPoseTo(wTwi,
      H2 ? &aTai_H_wTwa : nullptr,
      H1 ? &aTai_H_wTwi : nullptr);

  // Transform the plane measurement into sensor frame.
  const OrientedPlane3 i_plane = a_plane.transform(aTai,
      H2 ? &predicted_H_plane : nullptr,
      (H1 || H3) ? &predicted_H_aTai  : nullptr);

  // Calculate the error between measured and estimated planes in sensor frame.
  const Vector3 err = measured_p_.errorVector(i_plane,
    {}, (H1 || H2 || H3) ? &error_H_predicted : nullptr);

  // Apply the chain rule to calculate the derivatives.
  if (H1) {
    *H1 = error_H_predicted * predicted_H_aTai * aTai_H_wTwi;
  }

  if (H2) {
    *H2 = error_H_predicted * predicted_H_aTai * aTai_H_wTwa;
  }

  if (H3) {
    *H3 = error_H_predicted * predicted_H_plane;
  }

  return err;
}

}  // namespace gtsam
