/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   AttitudeFactor.cpp
 *  @author Frank Dellaert
 *  @brief  Implementation file for Attitude factor
 *  @date   January 28, 2014
 **/

#include "AttitudeFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void AttitudeFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "AttitudeFactor on " << keyFormatter(this->key()) << "\n";
  z_.print("  measured direction: ");
  ref_.print("  reference direction: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool AttitudeFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && this->z_.equals(e->z_, tol)
      && this->ref_.equals(e->ref_, tol);
}

//***************************************************************************
Vector AttitudeFactor::evaluateError(const Pose3& p,
    boost::optional<Matrix&> H) const {
  const Rot3& R = p.rotation();
  if (H) {
    Matrix D_q_R, D_e_q;
    Sphere2 q = R.rotate(z_, D_q_R);
    Vector e = ref_.error(q, D_e_q);
    H->resize(2, 6);
    H->block < 2, 3 > (0, 0) = D_e_q * D_q_R;
    H->block < 2, 3 > (0, 3) << Matrix::Zero(2, 3);
    return e;
  } else {
    Sphere2 q = R * z_;
    return ref_.error(q);
  }
}

//***************************************************************************

}/// namespace gtsam
