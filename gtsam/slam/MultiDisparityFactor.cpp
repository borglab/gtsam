/*
 * MultiDisparityFactor.cpp
 *
 *  Created on: Jan 30, 2014
 *      Author: nsrinivasan7
 */


#include "MultiDisparityFactor.h"
#include <gtsam/nonlinear/NonlinearFactor.h>


using namespace std;

namespace gtsam {

//***************************************************************************

void MultiDisparityFactor::print(const string& s) const {
  cout << "Prior Factor on " << landmarkKey_ << "\n";
  cout << "Measured Disparities : \n " << disparities_ << "\n";
  this->noiseModel_->print("  Noise model: ");
};

//***************************************************************************

Vector MultiDisparityFactor::evaluateError(const OrientedPlane3& plane,
    boost::optional<Matrix&> H) const {

  return Vector_(3,1,1,1);
};

}
