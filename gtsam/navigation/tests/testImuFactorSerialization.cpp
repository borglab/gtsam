/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactor.cpp
 * @brief   Unit test for ImuFactor
 * @author  Luca Carlone
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Stephen Williams
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>
#include <fstream>

using namespace std;
using namespace gtsam;

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained,
                        "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal,
                        "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian,
                        "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,
                        "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

TEST(ImuFactor, serialization) {
  using namespace gtsam::serializationTestHelpers;

  // Create default parameters with Z-down and above noise paramaters
  auto p = PreintegrationParams::MakeSharedD(9.81);
  p->body_P_sensor = Pose3(Rot3::Ypr(0, 0, M_PI), Point3(0,0,0));
  p->accelerometerCovariance = 1e-7 * I_3x3;
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  p->integrationCovariance = 1e-9 * I_3x3;

  const double deltaT = 0.005;
  const imuBias::ConstantBias priorBias(
      Vector3(0, 0, 0), Vector3(0, 0.01, 0));  // Biases (acc, rot)

  PreintegratedImuMeasurements pim(p, priorBias);

  // measurements are needed for non-inf noise model, otherwise will throw err
  // when deserialize
  const Vector3 measuredOmega(0, 0.01, 0);
  const Vector3 measuredAcc(0, 0, -9.81);

  for (int j = 0; j < 200; ++j)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  ImuFactor factor(1, 2, 3, 4, 5, pim);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
