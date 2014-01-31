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

  for(int i = 0; i < disparities_.rows(); i++) {
    cout << "Disparity @ (" << uv_(i,0) << ", " << uv_(i,1) << ") = " << disparities_(i) << "\n";
  }

  cameraPose_.print("Camera Pose ");
  this->noiseModel_->print("  noise model: ");
  cout << "\n";
};

//***************************************************************************

Vector MultiDisparityFactor::evaluateError(const OrientedPlane3& plane,
    boost::optional<Matrix&> H) const {


};

}
