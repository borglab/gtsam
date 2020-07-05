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

