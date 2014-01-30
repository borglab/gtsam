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
  cout << "Prior Factor on " << landmarkSymbol_ << "\n";
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
    H->resize(2,4);
  }

}
}

