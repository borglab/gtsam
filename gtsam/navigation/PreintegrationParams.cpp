/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationParams.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "PreintegrationParams.h"

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
void PreintegrationParams::print(const string& s) const {
  PreintegratedRotationParams::print(s);
  cout << "accelerometerCovariance:\n[\n" << accelerometerCovariance << "\n]"
       << endl;
  cout << "integrationCovariance:\n[\n" << integrationCovariance << "\n]"
       << endl;
  if (omegaCoriolis && use2ndOrderCoriolis)
    cout << "Using 2nd-order Coriolis" << endl;
  if (body_P_sensor) body_P_sensor->print("    ");
  cout << "n_gravity = (" << n_gravity.transpose() << ")" << endl;
}

//------------------------------------------------------------------------------
bool PreintegrationParams::equals(const PreintegratedRotationParams& other,
                                  double tol) const {
  auto e = dynamic_cast<const PreintegrationParams*>(&other);
  return e != nullptr && PreintegratedRotationParams::equals(other, tol) &&
         use2ndOrderCoriolis == e->use2ndOrderCoriolis &&
         equal_with_abs_tol(accelerometerCovariance, e->accelerometerCovariance,
                            tol) &&
         equal_with_abs_tol(integrationCovariance, e->integrationCovariance,
                            tol) &&
         equal_with_abs_tol(n_gravity, e->n_gravity, tol);
}

}  // namespace gtsam
