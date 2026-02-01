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
         << keyFormatter(key<1>()) << "Barometric Bias on "
         << keyFormatter(key<2>()) << "\n";

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
                                       OptionalMatrixType H,
                                       OptionalMatrixType H2) const {
    Matrix tH;
    Vector ret = (Vector(1) << (p.translation(tH).z() + bias - nT_)).finished();
    if (H) (*H) = tH.block<1, 6>(2, 0);
    if (H2) (*H2) = (Matrix(1, 1) << 1.0).finished();
    return ret;
}

}  // namespace gtsam
