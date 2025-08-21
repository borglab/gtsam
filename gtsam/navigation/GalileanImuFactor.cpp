/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanImuFactor.cpp
 * @brief   Implementation of Delama et al. IMU Factor
 * @author  Frank Dellaert
 * @author  Giulio Delama
 */

#include <gtsam/navigation/GalileanImuFactor.h>

namespace gtsam {

// -- Constructors ------------------------------------------------------------
PreintegratedImuMeasurementsG::PreintegratedImuMeasurementsG(
    const std::shared_ptr<Params>& p, const imuBias::ConstantBias& biasHat)
    : Base(p, biasHat), p_(p) {}

// -- Integration API ---------------------------------------------------------
void PreintegratedImuMeasurementsG::resetIntegration() {
  throw std::runtime_error("Not implemented: resetIntegration");
}

void PreintegratedImuMeasurementsG::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  throw std::runtime_error("Not implemented: integrateMeasurement");
}

// -- print / equals ----------------------------------------------------------
void PreintegratedImuMeasurementsG::print(const std::string& s) const {
  throw std::runtime_error("Not implemented: print");
}
bool PreintegratedImuMeasurementsG::equals(
    const PreintegratedImuMeasurementsG& o, double tol) const {
  throw std::runtime_error("Not implemented: equals");
}

}  // namespace gtsam
