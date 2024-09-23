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
Vector AttitudeFactor::attitudeError(const Rot3& nRb,
    OptionalJacobian<2, 3> H) const {
  if (H) {
    Matrix23 D_nRef_R;
    Matrix22 D_e_nRef;
    Unit3 nRef = nRb.rotate(bRef_, D_nRef_R);
    Vector e = nZ_.error(nRef, D_e_nRef);

    (*H) = D_e_nRef * D_nRef_R;
    return e;
  } else {
    Unit3 nRef = nRb * bRef_;
    return nZ_.error(nRef);
  }
}

//***************************************************************************
void Rot3AttitudeFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "Rot3AttitudeFactor on "
       << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Rot3AttitudeFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}

//***************************************************************************
void Pose3AttitudeFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "Pose3AttitudeFactor on " << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Pose3AttitudeFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}

//***************************************************************************

}/// namespace gtsam
