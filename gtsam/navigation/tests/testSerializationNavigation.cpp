/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSerializationNavigation.cpp
 * @brief   serialization tests for navigation
 * @author  Luca Carlone
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Stephen Williams
 * @author  Varun Agrawal
 * @date    February 2022
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <fstream>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

BOOST_CLASS_EXPORT_GUID(noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(noiseModel::Gaussian, "gtsam_noiseModel_Gaussian")
BOOST_CLASS_EXPORT_GUID(noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic")
BOOST_CLASS_EXPORT_GUID(SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(SharedDiagonal, "gtsam_SharedDiagonal")
BOOST_CLASS_EXPORT_GUID(PreintegratedImuMeasurements, "gtsam_PreintegratedImuMeasurements")
BOOST_CLASS_EXPORT_GUID(PreintegrationCombinedParams, "gtsam_PreintegrationCombinedParams")
BOOST_CLASS_EXPORT_GUID(PreintegratedCombinedMeasurements, "gtsam_PreintegratedCombinedMeasurements")

template <typename P>
P getPreintegratedMeasurements() {
  // Create default parameters with Z-down and above noise paramaters
  auto p = P::Params::MakeSharedD(9.81);
  p->body_P_sensor = Pose3(Rot3::Ypr(0, 0, M_PI), Point3(0, 0, 0));
  p->accelerometerCovariance = 1e-7 * I_3x3;
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  p->integrationCovariance = 1e-9 * I_3x3;

  const double deltaT = 0.005;

  // Biases (acc, rot)
  const imuBias::ConstantBias priorBias(Vector3(0, 0, 0), Vector3(0, 0.01, 0));

  P pim(p, priorBias);

  // measurements are needed for non-inf noise model, otherwise will throw error
  // when deserialize
  const Vector3 measuredOmega(0, 0.01, 0);
  const Vector3 measuredAcc(0, 0, -9.81);

  for (int j = 0; j < 200; ++j)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  return pim;
}

/* ************************************************************************* */
TEST(ImuFactor, serialization) {
  auto pim = getPreintegratedMeasurements<PreintegratedImuMeasurements>();

  EXPECT(equalsObj<PreintegratedImuMeasurements>(pim));
  EXPECT(equalsXML<PreintegratedImuMeasurements>(pim));
  EXPECT(equalsBinary<PreintegratedImuMeasurements>(pim));

  ImuFactor factor(1, 2, 3, 4, 5, pim);

  EXPECT(equalsObj<ImuFactor>(factor));
  EXPECT(equalsXML<ImuFactor>(factor));
  EXPECT(equalsBinary<ImuFactor>(factor));
}

/* ************************************************************************* */
TEST(ImuFactor2, serialization) {
  auto pim = getPreintegratedMeasurements<PreintegratedImuMeasurements>();

  ImuFactor2 factor(1, 2, 3, pim);

  EXPECT(equalsObj<ImuFactor2>(factor));
  EXPECT(equalsXML<ImuFactor2>(factor));
  EXPECT(equalsBinary<ImuFactor2>(factor));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, Serialization) {
  auto pim = getPreintegratedMeasurements<PreintegratedCombinedMeasurements>();

  EXPECT(equalsObj<PreintegratedCombinedMeasurements>(pim));
  EXPECT(equalsXML<PreintegratedCombinedMeasurements>(pim));
  EXPECT(equalsBinary<PreintegratedCombinedMeasurements>(pim));

  const CombinedImuFactor factor(1, 2, 3, 4, 5, 6, pim);

  EXPECT(equalsObj<CombinedImuFactor>(factor));
  EXPECT(equalsXML<CombinedImuFactor>(factor));
  EXPECT(equalsBinary<CombinedImuFactor>(factor));
}

/* ************************************************************************* */
TEST(Rot3AttitudeFactor, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Rot3AttitudeFactor factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
TEST(Pose3AttitudeFactor, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Pose3AttitudeFactor factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
