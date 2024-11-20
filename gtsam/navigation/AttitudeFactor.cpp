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
    Matrix23 D_nRotated_R;
    Matrix22 D_e_nRotated;
    Unit3 nRotated = nRb.rotate(bMeasured_, D_nRotated_R);
    Vector e = nRef_.error(nRotated, D_e_nRotated);

    (*H) = D_e_nRotated * D_nRotated_R;
    return e;
  } else {
    Unit3 nRotated = nRb * bMeasured_;
    return nRef_.error(nRotated);
  }
}

//***************************************************************************
void Rot3AttitudeFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << (s.empty() ? "" : s + " ") << "Rot3AttitudeFactor on "
       << keyFormatter(this->key()) << "\n";
  nRef_.print("  reference direction in nav frame: ");
  bMeasured_.print("  measured direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Rot3AttitudeFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && this->nRef_.equals(e->nRef_, tol)
      && this->bMeasured_.equals(e->bMeasured_, tol);
}

//***************************************************************************
void Pose3AttitudeFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "Pose3AttitudeFactor on " << keyFormatter(this->key()) << "\n";
  nRef_.print("  reference direction in nav frame: ");
  bMeasured_.print("  measured direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Pose3AttitudeFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) && this->nRef_.equals(e->nRef_, tol)
      && this->bMeasured_.equals(e->bMeasured_, tol);
}

//***************************************************************************

}/// namespace gtsam
