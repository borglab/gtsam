/*
 * OrientedPlane3Factor.cpp
 *
 *  Created on: Jan 29, 2014
 *  Author: Natesh Srinivasan
 */

#include "OrientedPlane3Factor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void OrientedPlane3Factor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << (s == "" ? "" : "\n");
  cout << "OrientedPlane3Factor Factor (" << keyFormatter(key<1>()) << ", "
       << keyFormatter(key<2>()) << ")\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
Vector OrientedPlane3Factor::evaluateError(const Pose3& pose,
    const OrientedPlane3& plane, OptionalMatrixType H1,
    OptionalMatrixType H2) const {
  Matrix36 predicted_H_pose;
  Matrix33 predicted_H_plane, error_H_predicted;

  OrientedPlane3 predicted_plane = plane.transform(pose,
    H2 ? &predicted_H_plane : nullptr, H1 ? &predicted_H_pose  : nullptr);

  Vector3 err = predicted_plane.errorVector(
      measured_p_, (H1 || H2) ? &error_H_predicted : nullptr);

  // Apply the chain rule to calculate the derivatives.
  if (H1) {
    *H1 = error_H_predicted * predicted_H_pose;
  }
  if (H2) {
    *H2 = error_H_predicted * predicted_H_plane;
  }

  return err;
}

//***************************************************************************
void OrientedPlane3DirectionPrior::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << (s == "" ? "" : "\n");
  cout << s << "Prior Factor on " << keyFormatter(key()) << "\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool OrientedPlane3DirectionPrior::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol)
      && this->measured_p_.equals(e->measured_p_, tol);
}

//***************************************************************************
Vector OrientedPlane3DirectionPrior::evaluateError(
    const OrientedPlane3& plane, OptionalMatrixType H) const {
  Unit3 n_hat_p = measured_p_.normal();
  Unit3 n_hat_q = plane.normal();
  Matrix2 H_p;
  Vector e = n_hat_p.error(n_hat_q, H ? &H_p : nullptr);
  if (H) {
    H->resize(2, 3);
    *H << H_p, Z_2x1;
  }
  return e;
}

}  // namespace gtsam
