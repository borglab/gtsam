/*
 * OrientedPlane3Factor.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: Natesh Srinivasan
 */

#include "OrientedPlane3Factor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void OrientedPlane3Factor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << "OrientedPlane3Factor Factor on " << landmarkKey_ << "\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
Vector OrientedPlane3Factor::evaluateError(const Pose3& pose,
    const OrientedPlane3& plane, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2) const {
  Vector err(3);

  if (H1 || H2) {
    Matrix36 predicted_H_pose;
    Matrix33 predicted_H_plane, error_H_predicted;

    OrientedPlane3 predicted_plane = plane.transform(pose, predicted_H_plane, predicted_H_pose);
    err << predicted_plane.error(measured_p_, error_H_predicted);

    // Apply the chain rule to calculate the derivatives.
    if (H1) {
      *H1 = error_H_predicted * predicted_H_pose;
    }
    if (H2) {
      *H2 = error_H_predicted * predicted_H_plane;
    }
  } else {
    OrientedPlane3 predicted_plane = plane.transform(pose);
    err << predicted_plane.error(measured_p_);
  }
  return (err);
}

//***************************************************************************
void OrientedPlane3DirectionPrior::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << "Prior Factor on " << landmarkKey_ << "\n";
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

Vector OrientedPlane3DirectionPrior::evaluateError(const OrientedPlane3& plane,
    boost::optional<Matrix&> H) const {

  if (H) {
    Matrix H_p;
    Unit3 n_hat_p = measured_p_.normal();
    Unit3 n_hat_q = plane.normal();
    Vector e = n_hat_p.error(n_hat_q, H_p);
    H->resize(2, 3);
    H->block<2, 2>(0, 0) << H_p;
    H->block<2, 1>(0, 2) << Z_2x1;
    return e;
  } else {
    Unit3 n_hat_p = measured_p_.normal();
    Unit3 n_hat_q = plane.normal();
    Vector e = n_hat_p.error(n_hat_q);
    return e;
  }

}
}

