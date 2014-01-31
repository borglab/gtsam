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

void OrientedPlane3DirectionPrior::print(const string& s) const {
  cout << "Prior Factor on " << landmarkKey_ << "\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************

bool OrientedPlane3DirectionPrior::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && this->measured_p_.equals(e->measured_p_, tol);
}

//***************************************************************************

Vector OrientedPlane3DirectionPrior::evaluateError(const OrientedPlane3& plane,
    boost::optional<Matrix&> H) const {

  if(H) {
    Matrix H_p;
    Sphere2 n_hat_p = measured_p_.normal();
    Sphere2 n_hat_q = plane.normal();
    Vector e = n_hat_p.error(n_hat_q,H_p);
    H->resize(2,3);
    H->block <2,2>(0,0) << H_p;
    H->block <2,1>(0,2) << Matrix::Zero(2, 1);
    return e;
  } else {
    Sphere2 n_hat_p = measured_p_.normal();
    Sphere2 n_hat_q = plane.normal();
    Vector e = n_hat_p.error(n_hat_q);
    return e;
  }

}
}

