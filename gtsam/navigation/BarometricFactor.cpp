/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   BarometricFactor.cpp
 *  @author Peter Milani
 *  @brief  Implementation file for Barometric factor
 *  @date   December 16, 2021
 **/

#include "BarometricFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void BarometricFactor::print(const string& s,
                             const KeyFormatter& keyFormatter) const {
    cout << (s.empty() ? "" : s + " ") << "Barometric Factor on "
         << keyFormatter(key1()) << "Barometric Bias on "
         << keyFormatter(key2()) << "\n";

    cout << "  Baro measurement: " << nT_ << "\n";
    noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool BarometricFactor::equals(const NonlinearFactor& expected,
                              double tol) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<double>::Equals(nT_, e->nT_, tol);
}

//***************************************************************************
Vector BarometricFactor::evaluateError(const Pose3& p, const double& bias,
                                       boost::optional<Matrix&> H,
                                       boost::optional<Matrix&> H2) const {
    if (H2) (*H2) = (Matrix(1, 1) << 1.0).finished();
    if (H) (*H) = (Matrix(1, 6) << 0., 0., 0., 0., 0., 1.).finished();
    return (Vector(1) << (p.translation().z() + bias - nT_)).finished();
}

}  // namespace gtsam
