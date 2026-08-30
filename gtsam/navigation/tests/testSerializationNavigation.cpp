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
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
#include <gtsam/navigation/DopplerFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuFactorWithGravity.h>
#include <gtsam/navigation/PseudorangeFactor.h>

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
BOOST_CLASS_EXPORT_GUID(PreintegratedImuMeasurements,
                        "gtsam_PreintegratedImuMeasurements")
BOOST_CLASS_EXPORT_GUID(PreintegrationCombinedParams,
                        "gtsam_PreintegrationCombinedParams")
BOOST_CLASS_EXPORT_GUID(PreintegratedCombinedMeasurements,
                        "gtsam_PreintegratedCombinedMeasurements")
BOOST_CLASS_EXPORT_GUID(PreintegratedImuMeasurementsG,
                        "gtsam_PreintegratedImuMeasurementsG")
BOOST_CLASS_EXPORT_GUID(GalileanImuFactor, "gtsam_GalileanImuFactor")
BOOST_CLASS_EXPORT_GUID(PreintegratedCombinedMeasurementsG,
                        "gtsam_PreintegratedCombinedMeasurementsG")
BOOST_CLASS_EXPORT_GUID(GalileanCombinedImuFactor,
                        "gtsam_GalileanCombinedImuFactor")

/* ************************************************************************* */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
// Legacy archives keep the ignored flag even though it no longer affects
// equality or prediction.
TEST(PreintegrationParams, LegacySecondOrderFlagSerialization) {
  PreintegrationParams input(Vector3(0.1, -0.2, -9.8));
  input.omegaCoriolis = Vector3(1e-5, -2e-5, 7e-5);
  input.setUse2ndOrderCoriolis(true);

  PreintegrationParams output;
  roundtrip(input, output);
  EXPECT(output.getUse2ndOrderCoriolis());

  PreintegrationParams semanticallyEquivalent = input;
  semanticallyEquivalent.setUse2ndOrderCoriolis(false);
  EXPECT(input.equals(semanticallyEquivalent, 1e-9));
}
#endif

/* ************************************************************************* */
TEST(AHRSFactor, Serialization) {
  auto params = std::make_shared<PreintegratedRotationParams>();
  params->gyroscopeCovariance = 1e-8 * I_3x3;
  PreintegratedAhrsMeasurements pim(params);
  pim.integrateMeasurement(Vector3(0.1, -0.2, 0.3), 0.01);

  EXPECT(equalsObj<PreintegratedAhrsMeasurements>(pim));
  EXPECT(equalsXML<PreintegratedAhrsMeasurements>(pim));
  EXPECT(equalsBinary<PreintegratedAhrsMeasurements>(pim));

  const AHRSFactor factor(1, 2, 3, pim);
  EXPECT(equalsObj<AHRSFactor>(factor));
  EXPECT(equalsXML<AHRSFactor>(factor));
  EXPECT(equalsBinary<AHRSFactor>(factor));
}

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
TEST(ImuFactorWithGravity, serialization) {
  auto pim = getPreintegratedMeasurements<PreintegratedImuMeasurements>();

  ImuFactorWithGravityDirection direction(1, 2, 3, 4, 5, 6, pim, 9.81);
  EXPECT(equalsObj<ImuFactorWithGravityDirection>(direction));
  EXPECT(equalsXML<ImuFactorWithGravityDirection>(direction));
  EXPECT(equalsBinary<ImuFactorWithGravityDirection>(direction));

  ImuFactorWithGravityVector vector(1, 2, 3, 4, 5, 6, pim);
  EXPECT(equalsObj<ImuFactorWithGravityVector>(vector));
  EXPECT(equalsXML<ImuFactorWithGravityVector>(vector));
  EXPECT(equalsBinary<ImuFactorWithGravityVector>(vector));
}

/* ************************************************************************* */
// Round-trips the NavState-based gravity factors; the non-default magnitude
// verifies it is restored from the archive, not recomputed from the params.
TEST(ImuFactor2WithGravity, serialization) {
  auto pim = getPreintegratedMeasurements<PreintegratedImuMeasurements>();

  ImuFactor2WithGravityDirection direction(1, 2, 3, 4, pim, 1.62);
  EXPECT(equalsObj<ImuFactor2WithGravityDirection>(direction));
  EXPECT(equalsXML<ImuFactor2WithGravityDirection>(direction));
  EXPECT(equalsBinary<ImuFactor2WithGravityDirection>(direction));

  ImuFactor2WithGravityVector vector(1, 2, 3, 4, pim);
  EXPECT(equalsObj<ImuFactor2WithGravityVector>(vector));
  EXPECT(equalsXML<ImuFactor2WithGravityVector>(vector));
  EXPECT(equalsBinary<ImuFactor2WithGravityVector>(vector));
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
TEST(CombinedImuFactorWithGravity, Serialization) {
  auto pim = getPreintegratedMeasurements<PreintegratedCombinedMeasurements>();

  CombinedImuFactorWithGravityDirection direction(1, 2, 3, 4, 5, 6, 7, pim, 9.81);
  EXPECT(equalsObj<CombinedImuFactorWithGravityDirection>(direction));
  EXPECT(equalsXML<CombinedImuFactorWithGravityDirection>(direction));
  EXPECT(equalsBinary<CombinedImuFactorWithGravityDirection>(direction));

  CombinedImuFactorWithGravityVector vector(1, 2, 3, 4, 5, 6, 7, pim);
  EXPECT(equalsObj<CombinedImuFactorWithGravityVector>(vector));
  EXPECT(equalsXML<CombinedImuFactorWithGravityVector>(vector));
  EXPECT(equalsBinary<CombinedImuFactorWithGravityVector>(vector));
}

/* ************************************************************************* */
namespace lie_group_serialization {

using Pim = PreintegratedImuMeasurementsT<LieGroupPreintegration>;
using CombinedPim = PreintegratedCombinedMeasurementsT<LieGroupPreintegration>;
using StandardFactor = ImuFactorT<Pim>;
using NavStateFactor = ImuFactor2T<Pim>;
using GravityDirectionFactor = ImuFactorWithGravityT<Pim, Unit3>;
using GravityVectorFactor = ImuFactorWithGravityT<Pim, Point3>;
using NavStateGravityDirectionFactor = ImuFactor2WithGravityT<Pim, Unit3>;
using NavStateGravityVectorFactor = ImuFactor2WithGravityT<Pim, Point3>;
using CombinedFactor = CombinedImuFactorT<CombinedPim>;
using CombinedGravityDirectionFactor =
    CombinedImuFactorWithGravityT<CombinedPim, Unit3>;
using CombinedGravityVectorFactor =
    CombinedImuFactorWithGravityT<CombinedPim, Point3>;

// Verifies explicit Lie-backed measurements and every factor family round-trip.
TEST(LieGroupPreintegration, Serialization) {
  const Pim pim = getPreintegratedMeasurements<Pim>();
  EXPECT(equalsObj(pim));
  EXPECT(equalsXML(pim));
  EXPECT(equalsBinary(pim));

  const StandardFactor standard(1, 2, 3, 4, 5, pim);
  EXPECT(equalsObj(standard));
  EXPECT(equalsXML(standard));
  EXPECT(equalsBinary(standard));

  const NavStateFactor navState(1, 2, 3, pim);
  EXPECT(equalsObj(navState));
  EXPECT(equalsXML(navState));
  EXPECT(equalsBinary(navState));

  const GravityDirectionFactor gravityDirection(1, 2, 3, 4, 5, 6, pim, 9.81);
  EXPECT(equalsObj(gravityDirection));
  EXPECT(equalsXML(gravityDirection));
  EXPECT(equalsBinary(gravityDirection));

  const GravityVectorFactor gravityVector(1, 2, 3, 4, 5, 6, pim);
  EXPECT(equalsObj(gravityVector));
  EXPECT(equalsXML(gravityVector));
  EXPECT(equalsBinary(gravityVector));

  const NavStateGravityDirectionFactor navStateGravityDirection(1, 2, 3, 4,
                                                                pim, 9.81);
  EXPECT(equalsObj(navStateGravityDirection));
  EXPECT(equalsXML(navStateGravityDirection));
  EXPECT(equalsBinary(navStateGravityDirection));

  const NavStateGravityVectorFactor navStateGravityVector(1, 2, 3, 4, pim);
  EXPECT(equalsObj(navStateGravityVector));
  EXPECT(equalsXML(navStateGravityVector));
  EXPECT(equalsBinary(navStateGravityVector));

  const CombinedPim combinedPim = getPreintegratedMeasurements<CombinedPim>();
  EXPECT(equalsObj(combinedPim));
  EXPECT(equalsXML(combinedPim));
  EXPECT(equalsBinary(combinedPim));

  const CombinedFactor combined(1, 2, 3, 4, 5, 6, combinedPim);
  EXPECT(equalsObj(combined));
  EXPECT(equalsXML(combined));
  EXPECT(equalsBinary(combined));

  const CombinedGravityDirectionFactor combinedGravityDirection(
      1, 2, 3, 4, 5, 6, 7, combinedPim, 9.81);
  EXPECT(equalsObj(combinedGravityDirection));
  EXPECT(equalsXML(combinedGravityDirection));
  EXPECT(equalsBinary(combinedGravityDirection));

  const CombinedGravityVectorFactor combinedGravityVector(1, 2, 3, 4, 5, 6, 7,
                                                          combinedPim);
  EXPECT(equalsObj(combinedGravityVector));
  EXPECT(equalsXML(combinedGravityVector));
  EXPECT(equalsBinary(combinedGravityVector));
}

}  // namespace lie_group_serialization
/* ************************************************************************* */

/* ************************************************************************* */
namespace galilean_serialization {

// Verifies the Galilean PIM and its public factor round-trip through every
// supported archive format.
TEST(GalileanPreintegration, Serialization) {
  const PreintegratedImuMeasurementsG pim =
      getPreintegratedMeasurements<PreintegratedImuMeasurementsG>();
  EXPECT(equalsObj(pim));
  EXPECT(equalsXML(pim));
  EXPECT(equalsBinary(pim));

  const GalileanImuFactor factor(1, 2, 3, 4, 5, pim);
  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));

  const PreintegratedCombinedMeasurementsG combinedPim =
      getPreintegratedMeasurements<PreintegratedCombinedMeasurementsG>();
  EXPECT(equalsObj(combinedPim));
  EXPECT(equalsXML(combinedPim));
  EXPECT(equalsBinary(combinedPim));

  const GalileanCombinedImuFactor combinedFactor(1, 2, 3, 4, 5, 6, combinedPim);
  EXPECT(equalsObj(combinedFactor));
  EXPECT(equalsXML(combinedFactor));
  EXPECT(equalsBinary(combinedFactor));
}

}  // namespace galilean_serialization
/* ************************************************************************* */

/* ************************************************************************* */
TEST(AttitudeFactorRot3, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  AttitudeFactor<Rot3> factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
TEST(AttitudeFactorPose3, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  AttitudeFactor<Pose3> factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
TEST(AttitudeFactorNavState, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  AttitudeFactor<NavState> factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
TEST(AttitudeFactorGal3, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  AttitudeFactor<Gal3> factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
TEST(AttitudeFactorSe23, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  AttitudeFactor<Se23> factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
TEST(AttitudeFactorExtendedPose3d, Serialization) {
  Unit3 nDown(0, 0, -1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  AttitudeFactor<ExtendedPose3d> factor(0, nDown, model);

  EXPECT(serializationTestHelpers::equalsObj(factor));
  EXPECT(serializationTestHelpers::equalsXML(factor));
  EXPECT(serializationTestHelpers::equalsBinary(factor));
}

/* ************************************************************************* */
// GPS lever-arm factors: verify the measurement (nT_) and lever arm (bL_)
// survive an obj/XML/binary serialization round-trip.
TEST(GPSFactorArm, Serialization) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactorArm factor(0, Point3(1.0, 2.0, 3.0), Point3(0.1, 0.2, 0.3), model);
  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
TEST(GPSFactorArmCalib, Serialization) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactorArmCalib factor(0, 1, Point3(1.0, 2.0, 3.0), model);
  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
TEST(GPSFactor2Arm, Serialization) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor2Arm factor(0, Point3(1.0, 2.0, 3.0), Point3(0.1, 0.2, 0.3), model);
  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
TEST(GPSFactor2ArmCalib, Serialization) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);
  GPSFactor2ArmCalib factor(0, 1, Point3(1.0, 2.0, 3.0), model);
  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
// GNSS factors: verify the measurement and geometry members survive an
// obj/XML/binary serialization round-trip. (The base object is serialized with
// a tag valid for XML archives.)
namespace {
const Point3 kSat1(1.5e7, -1.2e7, 2.0e7);
const Point3 kSat2(1.6e7, -1.1e7, 2.1e7);
const Point3 kSat3(1.4e7, -1.3e7, 1.9e7);
const Point3 kSat4(1.7e7, -1.0e7, 2.2e7);
const Point3 kBase(-2.69e6, -4.29e6, 3.86e6);
const Point3 kLever(0.10, 0.20, 0.30);
const SharedNoiseModel kGnss = noiseModel::Isotropic::Sigma(1, 0.5);
const double kPr = 2.1e7, kLam = 0.19;
}  // namespace

/* ************************************************************************* */
TEST(PseudorangeFactor, Serialization) {
  PseudorangeFactor f(0, 1, kPr, kSat1, 1e-4, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(PseudorangeFactorArm, Serialization) {
  PseudorangeFactorArm f(0, 1, kPr, kSat1, kLever, 1e-4, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DifferentialPseudorangeFactor, Serialization) {
  DifferentialPseudorangeFactor f(0, 1, 2, kPr, kSat1, 1e-4, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DifferentialPseudorangeFactorArm, Serialization) {
  DifferentialPseudorangeFactorArm f(0, 1, 2, kPr, kSat1, kLever, 1e-4, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DoubleDifferencePseudorangeFactor, Serialization) {
  DoubleDifferencePseudorangeFactor f(0, kPr, kPr + 1, kPr + 2, kPr + 3, kSat1,
                                      kSat2, kSat3, kSat4, kBase, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DoubleDifferencePseudorangeFactorArm, Serialization) {
  DoubleDifferencePseudorangeFactorArm f(0, kPr, kPr + 1, kPr + 2, kPr + 3,
                                         kSat1, kSat2, kSat3, kSat4, kBase,
                                         kLever, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(CarrierPhaseFactor, Serialization) {
  CarrierPhaseFactor f(0, 1, 2, kPr, kSat1, 1e-4, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(CarrierPhaseFactorArm, Serialization) {
  CarrierPhaseFactorArm f(0, 1, 2, kPr, kSat1, kLever, 1e-4, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DoubleDifferenceCarrierPhaseFactor, Serialization) {
  DoubleDifferenceCarrierPhaseFactor f(0, 1, 2, kPr, kPr + 1, kPr + 2, kPr + 3,
                                       kSat1, kSat2, kSat3, kSat4, kBase, kLam,
                                       kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DoubleDifferenceCarrierPhaseFactorArm, Serialization) {
  DoubleDifferenceCarrierPhaseFactorArm f(0, 1, 2, kPr, kPr + 1, kPr + 2,
                                          kPr + 3, kSat1, kSat2, kSat3, kSat4,
                                          kBase, kLam, kLever, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DopplerFactor, Serialization) {
  DopplerFactor f(0, 1, 2, -1500.0, kLam, kSat1, Point3(-1200, 2400, 800),
                  kBase, 1.0, 1.2e-9, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DopplerFactorArm, Serialization) {
  DopplerFactorArm f(0, 1, 2, 3, -1500.0, kLam, kSat1,
                     Point3(-1200, 2400, 800), kBase, kLever,
                     Point3(0.02, -0.05, 0.1), 1.0, 1.2e-9, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(DopplerFactorArm, SerializationNavFrame) {
  DopplerFactorArm f(0, 1, 2, 3, -1500.0, kLam, kSat1,
                     Point3(-1200, 2400, 800), kBase, kLever,
                     Pose3(Rot3::RzRyRx(0.1, 0.4, -0.7), kBase),
                     Point3(0.02, -0.05, 0.1), 1.0, 1.2e-9, kGnss);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
// Undifferenced (PPP) GNSS factors: verify the measurement, geometry and
// tropo/iono coefficients survive an obj/XML/binary serialization round-trip.
namespace {
const Point3 kUSat(1.5e7, -1.2e7, 2.0e7);
const Point3 kULever(0.10, 0.20, 0.30);
const SharedNoiseModel kUModel = noiseModel::Isotropic::Sigma(1, 0.5);
const double kUPr = 2.1e7, kUmw = 3.2, kUmu = 1.55, kULam = 0.19;
}  // namespace

/* ************************************************************************* */
TEST(UndifferencedPseudorangeFactor, Serialization) {
  UndifferencedPseudorangeFactor f(0, 1, 2, 3, kUPr, kUSat, kUmw, kUmu, 1e-4,
                                   kUModel);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(UndifferencedPseudorangeFactorArm, Serialization) {
  UndifferencedPseudorangeFactorArm f(0, 1, 2, 3, kUPr, kUSat, kULever, kUmw,
                                      kUmu, 1e-4, kUModel);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(UndifferencedCarrierPhaseFactor, Serialization) {
  UndifferencedCarrierPhaseFactor f(0, 1, 2, 3, 4, kUPr, kUSat, kUmw, -kUmu,
                                    kULam, 1e-4, kUModel);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
TEST(UndifferencedCarrierPhaseFactorArm, Serialization) {
  UndifferencedCarrierPhaseFactorArm f(0, 1, 2, 3, 4, kUPr, kUSat, kULever, kUmw,
                                       -kUmu, kULam, 1e-4, kUModel);
  EXPECT(equalsObj(f));
  EXPECT(equalsXML(f));
  EXPECT(equalsBinary(f));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
