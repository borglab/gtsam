/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactorCovariance.cpp
 * @brief   Regression tests for the IMU factor residual covariance
 * @author  Frank Dellaert (with codex)
 * @date    August 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuFactorWithGravity.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>

#include "imuFactorTesting.h"

/* ************************************************************************* */
namespace imu_covariance {

constexpr double kDt = 0.02;
constexpr size_t kMeasurementCount = 60;
constexpr size_t kMonteCarloSamples = 5000;
constexpr uint_fast64_t kRandomSeed = 2384;
constexpr double kMonteCarloTolerance = 0.12;

const Vector3 kMeasuredAcceleration{0.3, -0.2, 9.7};
const Vector3 kMeasuredAngularVelocity{0.15, -0.10, 1.0};
const Vector3 kAccelerometerSigmas{0.02, 0.08, 0.15};
const Vector3 kGyroscopeSigmas{0.01, 0.04, 0.08};
const Vector3 kAccelerometerBiasRandomWalkSigmas{0.02, 0.05, 0.10};
const Vector3 kGyroscopeBiasRandomWalkSigmas{0.01, 0.03, 0.06};

using Matrix15 = Eigen::Matrix<double, 15, 15>;

using symbol_shorthand::G;

std::shared_ptr<PreintegrationParams> makeParams() {
  auto params = PreintegrationParams::MakeSharedD(10.0);
  params->accelerometerCovariance =
      kAccelerometerSigmas.cwiseProduct(kAccelerometerSigmas).asDiagonal();
  params->gyroscopeCovariance =
      kGyroscopeSigmas.cwiseProduct(kGyroscopeSigmas).asDiagonal();
  params->integrationCovariance = Z_3x3;
  return params;
}

template <class PIM>
struct PimBackend;

template <class Backend>
struct PimBackend<PreintegratedImuMeasurementsT<Backend>> {
  using Type = Backend;
};

template <class PIM>
bool matchesDensePropagation(bool displacedSensor) {
  auto params = makeParams();
  const Matrix3 accelerometerRoot{
      {0.02, 0.00, 0.00}, {0.01, 0.07, 0.00}, {-0.02, 0.03, 0.14}};
  const Matrix3 gyroscopeRoot{
      {0.01, 0.00, 0.00}, {-0.002, 0.04, 0.00}, {0.003, -0.01, 0.07}};
  params->accelerometerCovariance =
      accelerometerRoot * accelerometerRoot.transpose();
  params->gyroscopeCovariance = gyroscopeRoot * gyroscopeRoot.transpose();
  params->integrationCovariance = 1e-5 * I_3x3;
  if (displacedSensor) {
    params->body_P_sensor =
        Pose3(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(0.2, -0.1, 0.05));
  }

  using Backend = typename PimBackend<PIM>::Type;
  const imuBias::ConstantBias bias(Vector3(0.01, -0.02, 0.03),
                                   Vector3(-0.004, 0.005, -0.006));
  Backend backend(params, bias);
  PIM pim(params, bias);
  Matrix9 expected = Z_9x9;
  for (size_t sample = 0; sample < 25; ++sample) {
    const double scale = static_cast<double>(sample);
    const Vector3 acceleration =
        kMeasuredAcceleration + scale * Vector3(0.002, -0.001, 0.0005);
    const Vector3 angularVelocity =
        kMeasuredAngularVelocity + scale * Vector3(-0.0003, 0.0002, 0.0001);
    Matrix9 A;
    Matrix93 B, C;
    backend.update(acceleration, angularVelocity, kDt, &A, &B, &C);
    expected = A * expected * A.transpose();
    expected.noalias() +=
        B * (params->accelerometerCovariance / kDt) * B.transpose();
    expected.noalias() +=
        C * (params->gyroscopeCovariance / kDt) * C.transpose();
    expected.block<3, 3>(3, 3).noalias() += params->integrationCovariance * kDt;
    pim.integrateMeasurement(acceleration, angularVelocity, kDt);
  }
  return assert_equal(expected, pim.preintMeasCov(), 1e-12);
}

std::shared_ptr<PreintegrationCombinedParams> makeCombinedParams() {
  auto params = PreintegrationCombinedParams::MakeSharedD(10.0);
  params->accelerometerCovariance =
      kAccelerometerSigmas.cwiseProduct(kAccelerometerSigmas).asDiagonal();
  params->gyroscopeCovariance =
      kGyroscopeSigmas.cwiseProduct(kGyroscopeSigmas).asDiagonal();
  params->integrationCovariance = Z_3x3;
  params->biasAccCovariance =
      kAccelerometerBiasRandomWalkSigmas
          .cwiseProduct(kAccelerometerBiasRandomWalkSigmas)
          .asDiagonal();
  params->biasOmegaCovariance =
      kGyroscopeBiasRandomWalkSigmas
          .cwiseProduct(kGyroscopeBiasRandomWalkSigmas)
          .asDiagonal();
  return params;
}

template <class PIM>
struct CombinedPimBackend;

template <class Backend>
struct CombinedPimBackend<PreintegratedCombinedMeasurementsT<Backend>> {
  using Type = Backend;
};

template <class PIM>
bool matchesDenseCombinedPropagation(bool displacedSensor) {
  auto params = makeCombinedParams();
  const Matrix3 accelerometerRoot{
      {0.02, 0.00, 0.00}, {0.01, 0.07, 0.00}, {-0.02, 0.03, 0.14}};
  const Matrix3 gyroscopeRoot{
      {0.01, 0.00, 0.00}, {-0.002, 0.04, 0.00}, {0.003, -0.01, 0.07}};
  params->accelerometerCovariance =
      accelerometerRoot * accelerometerRoot.transpose();
  params->gyroscopeCovariance = gyroscopeRoot * gyroscopeRoot.transpose();
  params->integrationCovariance = 1e-5 * I_3x3;
  if (displacedSensor) {
    params->body_P_sensor =
        Pose3(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(0.2, -0.1, 0.05));
  }

  using Backend = typename CombinedPimBackend<PIM>::Type;
  const imuBias::ConstantBias bias(Vector3(0.01, -0.02, 0.03),
                                   Vector3(-0.004, 0.005, -0.006));
  Backend backend(params, bias);
  PIM pim(params, bias);
  Matrix15 expected = Matrix15::Zero();
  for (size_t sample = 0; sample < 25; ++sample) {
    const double scale = static_cast<double>(sample);
    const Vector3 acceleration =
        kMeasuredAcceleration + scale * Vector3(0.002, -0.001, 0.0005);
    const Vector3 angularVelocity =
        kMeasuredAngularVelocity + scale * Vector3(-0.0003, 0.0002, 0.0001);
    Matrix9 A;
    Matrix93 B, C;
    backend.update(acceleration, angularVelocity, kDt, &A, &B, &C);

    Matrix15 F = Matrix15::Zero();
    F.block<9, 9>(0, 0) = A;
    F.block<9, 3>(0, 9) = B;
    F.block<9, 3>(0, 12) = C;
    F.block<6, 6>(9, 9) = I_6x6;
    expected = F * expected * F.transpose();

    const Matrix3 scaledAccelerometerCovariance =
        params->accelerometerCovariance / kDt;
    const Matrix3 scaledGyroscopeCovariance = params->gyroscopeCovariance / kDt;
    expected.topLeftCorner<9, 9>().noalias() +=
        B * scaledAccelerometerCovariance * B.transpose();
    expected.topLeftCorner<9, 9>().noalias() +=
        C * scaledGyroscopeCovariance * C.transpose();
    expected.block<3, 3>(3, 3).noalias() += kDt * params->integrationCovariance;
    expected.block<3, 3>(9, 9).noalias() += kDt * params->biasAccCovariance;
    expected.block<3, 3>(12, 12).noalias() += kDt * params->biasOmegaCovariance;
    pim.integrateMeasurement(acceleration, angularVelocity, kDt);
  }
  return assert_equal(expected, pim.preintMeasCov(), 1e-12);
}

template <class PIM>
PIM integrateIdealMeasurements(
    const std::shared_ptr<PreintegrationParams>& params) {
  PIM pim(params);
  for (size_t i = 0; i < kMeasurementCount; ++i) {
    pim.integrateMeasurement(kMeasuredAcceleration, kMeasuredAngularVelocity,
                             kDt);
  }
  return pim;
}

template <template <class> class PimTemplate, class PreintegrationType>
Matrix9 residualChartJacobian(const PimTemplate<PreintegrationType>& pim) {
  static_assert(
      std::is_same_v<PreintegrationType, ManifoldPreintegration> ||
          std::is_same_v<PreintegrationType, TangentPreintegration> ||
          std::is_same_v<PreintegrationType, LieGroupPreintegration> ||
          std::is_same_v<PreintegrationType, GalileanPreintegration>,
      "Unsupported IMU preintegration backend");
  if constexpr (std::is_same_v<PreintegrationType, TangentPreintegration>) {
    // Map additive [theta, position, velocity] perturbations into the
    // component-wise NavState chart used by the factor residual.
    Matrix9 jacobian = Z_9x9;
    jacobian.block<3, 3>(0, 0) =
        Rot3::ExpmapDerivative(pim.preintegrated().template head<3>());
    const Matrix3 deltaRotationTranspose = pim.deltaRij().matrix().transpose();
    jacobian.block<3, 3>(3, 3) = deltaRotationTranspose;
    jacobian.block<3, 3>(6, 6) = deltaRotationTranspose;
    return jacobian;
  } else {
    return I_9x9;
  }
}

// Galilean preintegration already exposes covariance in the factor chart.
Matrix9 residualChartJacobian(const PreintegratedImuMeasurementsG&) {
  return I_9x9;
}

template <class CombinedPIM>
CombinedPIM integrateIdealCombinedMeasurements(
    const std::shared_ptr<PreintegrationCombinedParams>& params) {
  CombinedPIM pim(params);
  for (size_t i = 0; i < kMeasurementCount; ++i) {
    pim.integrateMeasurement(kMeasuredAcceleration, kMeasuredAngularVelocity,
                             kDt);
  }
  return pim;
}

template <class PIM>
Matrix9 theoreticalResidualCovariance(const PIM& pim) {
  const Matrix9 jacobian = residualChartJacobian(pim);
  return jacobian * pim.preintMeasCov() * jacobian.transpose();
}

const NavState kErrorStateI(Rot3::Ypr(0.4, -0.2, 0.1), Point3(1.0, -2.0, 0.5),
                            Vector3(0.4, 0.2, -0.1));
const NavState kErrorStateJ(Rot3::Ypr(-0.2, 0.3, -0.1), Point3(-0.5, 1.2, 2.0),
                            Vector3(-0.3, 0.6, 0.2));
const imuBias::ConstantBias kErrorBias(Vector3(0.01, -0.02, 0.03),
                                       Vector3(-0.004, 0.005, -0.006));

// Fresh standard and Combined parameters must select Logmap uniformly.
TEST(ImuFactorErrorMode, FreshParamsDefaultToLogmap) {
  const PreintegrationParams direct(Vector3(0.0, 0.0, -9.81));
  const auto down = PreintegrationParams::MakeSharedD();
  const auto up = PreintegrationParams::MakeSharedU();
  const PreintegrationCombinedParams combinedDirect(
      Vector3(0.0, 0.0, -9.81));
  const auto combinedDown = PreintegrationCombinedParams::MakeSharedD();
  const auto combinedUp = PreintegrationCombinedParams::MakeSharedU();

  EXPECT(direct.getImuFactorErrorMode() == ImuFactorErrorMode::Logmap);
  EXPECT(down->getImuFactorErrorMode() == ImuFactorErrorMode::Logmap);
  EXPECT(up->getImuFactorErrorMode() == ImuFactorErrorMode::Logmap);
  EXPECT(combinedDirect.getImuFactorErrorMode() ==
         ImuFactorErrorMode::Logmap);
  EXPECT(combinedDown->getImuFactorErrorMode() ==
         ImuFactorErrorMode::Logmap);
  EXPECT(combinedUp->getImuFactorErrorMode() ==
         ImuFactorErrorMode::Logmap);
}

// Locks the compatibility policy independently of the runtime dispatch test.
TEST(ImuFactorErrorMode, LegacyBackendMapping) {
  EXPECT(!ManifoldPreintegration::kLegacyUsesLogmap);
  EXPECT(!TangentPreintegration::kLegacyUsesLogmap);
  EXPECT(LieGroupPreintegration::kLegacyUsesLogmap);
  EXPECT(GalileanPreintegration::kLegacyUsesLogmap);
}

template <class PIM>
bool usesLogmap(ImuFactorErrorMode mode) {
  switch (mode) {
    case ImuFactorErrorMode::Legacy:
      return PIM::kLegacyUsesLogmap;
    case ImuFactorErrorMode::ComponentWise:
      return false;
    case ImuFactorErrorMode::Logmap:
      return true;
  }
  throw std::invalid_argument("Unknown ImuFactorErrorMode");
}

template <class PIM>
Vector9 expectedError(const PIM& pim, ImuFactorErrorMode mode) {
  const NavState predicted = pim.predict(kErrorStateI, kErrorBias);
  return usesLogmap<PIM>(mode)
             ? kErrorStateJ.logmap(predicted)
             : internal::navStateComponentWiseLocalCoordinates(kErrorStateJ,
                                                               predicted);
}

// Checks every backend and runtime mode, including finite-residual Jacobians.
TEST_PIM(ImuFactorErrorMode, BackendSelectionAndJacobians) {
  const auto params = makeParams();
  const PIM pim = integrateIdealMeasurements<PIM>(params);
  EXPECT(params->getImuFactorErrorMode() == ImuFactorErrorMode::Logmap);

  const ImuFactor2T<PIM> defaultFactor(X(1), X(2), B(1), pim);
  const Vector9 defaultError =
      defaultFactor.evaluateError(kErrorStateI, kErrorStateJ, kErrorBias);
  EXPECT(assert_equal(expectedError(pim, ImuFactorErrorMode::Logmap),
                      defaultError, 1e-12));
  const ImuFactorT<PIM> defaultSplitFactor(X(1), V(1), X(2), V(2), B(1), pim);
  EXPECT(assert_equal(
      defaultError,
      defaultSplitFactor.evaluateError(
          kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
          kErrorStateJ.velocity(), kErrorBias),
      1e-12));

  const ImuFactorErrorMode modes[] = {ImuFactorErrorMode::Legacy,
                                      ImuFactorErrorMode::ComponentWise,
                                      ImuFactorErrorMode::Logmap};

  for (const ImuFactorErrorMode mode : modes) {
    params->setImuFactorErrorMode(mode);
    const ImuFactor2T<PIM> factor(X(1), X(2), B(1), pim);
    Matrix actualH1, actualH2, actualH3;
    const Vector9 actual =
        factor.evaluateError(kErrorStateI, kErrorStateJ, kErrorBias, &actualH1,
                             &actualH2, &actualH3);
    EXPECT(assert_equal(expectedError(pim, mode), actual, 1e-12));

    auto error = [&factor](const NavState& state_i, const NavState& state_j,
                           const imuBias::ConstantBias& bias) {
      return factor.evaluateError(state_i, state_j, bias);
    };
    EXPECT(assert_equal(
        numericalDerivative31(error, kErrorStateI, kErrorStateJ, kErrorBias),
        actualH1, 1e-7));
    EXPECT(assert_equal(
        numericalDerivative32(error, kErrorStateI, kErrorStateJ, kErrorBias),
        actualH2, 1e-7));
    EXPECT(assert_equal(
        numericalDerivative33(error, kErrorStateI, kErrorStateJ, kErrorBias),
        actualH3, 1e-7));

    const ImuFactorT<PIM> splitFactor(X(1), V(1), X(2), V(2), B(1), pim);
    EXPECT(assert_equal(
        actual,
        splitFactor.evaluateError(kErrorStateI.pose(), kErrorStateI.velocity(),
                                  kErrorStateJ.pose(), kErrorStateJ.velocity(),
                                  kErrorBias),
        1e-12));
  }
}

// Checks Combined factors use the same selected navigation residual.
TEST_PIM_WITH_COMBINED_BACKEND(ImuFactorErrorMode, CombinedSelection) {
  const auto params = makeCombinedParams();
  const CombinedPIM pim =
      integrateIdealCombinedMeasurements<CombinedPIM>(params);
  const imuBias::ConstantBias biasJ(Vector3(-0.02, 0.01, 0.04),
                                    Vector3(0.002, -0.003, 0.001));
  EXPECT(params->getImuFactorErrorMode() == ImuFactorErrorMode::Logmap);
  const CombinedImuFactorT<CombinedPIM> defaultFactor(
      X(1), V(1), X(2), V(2), B(1), B(2), pim);
  const Vector15 defaultError = defaultFactor.evaluateError(
      kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
      kErrorStateJ.velocity(), kErrorBias, biasJ);
  EXPECT(assert_equal(expectedError(pim, ImuFactorErrorMode::Logmap),
                      Vector9(defaultError.head<9>()), 1e-12));

  const ImuFactorErrorMode modes[] = {ImuFactorErrorMode::Legacy,
                                      ImuFactorErrorMode::ComponentWise,
                                      ImuFactorErrorMode::Logmap};

  for (const ImuFactorErrorMode mode : modes) {
    params->setImuFactorErrorMode(mode);
    const CombinedImuFactorT<CombinedPIM> factor(X(1), V(1), X(2), V(2), B(1),
                                                 B(2), pim);
    const Vector15 actual = factor.evaluateError(
        kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
        kErrorStateJ.velocity(), kErrorBias, biasJ);
    const Vector9 actualNavigationError = actual.head<9>();
    const Vector6 actualBiasError = actual.tail<6>();
    const Vector6 expectedBiasError = kErrorBias.vector() - biasJ.vector();
    EXPECT(
        assert_equal(expectedError(pim, mode), actualNavigationError, 1e-12));
    EXPECT(assert_equal(expectedBiasError, actualBiasError, 1e-12));
  }
}

// Checks both gravity-aware factor topologies for every standard backend.
TEST_PIM(ImuFactorErrorMode, GravityFactorSelection) {
  const auto params = makeParams();
  const PIM pim = integrateIdealMeasurements<PIM>(params);
  const ImuFactor2WithGravityT<PIM, Point3> gravityFactor(X(1), X(2), B(1),
                                                          G(1), pim);
  const ImuFactorWithGravityT<PIM, Point3> splitGravityFactor(
      X(1), V(1), X(2), V(2), B(1), G(1), pim);

  const Vector9 defaultGravityError = gravityFactor.evaluateError(
      kErrorStateI, kErrorStateJ, kErrorBias, Point3(params->n_gravity));
  EXPECT(assert_equal(expectedError(pim, ImuFactorErrorMode::Logmap),
                      defaultGravityError, 1e-12));
  EXPECT(assert_equal(
      defaultGravityError,
      splitGravityFactor.evaluateError(
          kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
          kErrorStateJ.velocity(), kErrorBias, Point3(params->n_gravity)),
      1e-12));

  const ImuFactorErrorMode modes[] = {ImuFactorErrorMode::Legacy,
                                      ImuFactorErrorMode::ComponentWise,
                                      ImuFactorErrorMode::Logmap};
  for (const ImuFactorErrorMode mode : modes) {
    params->setImuFactorErrorMode(mode);
    const Vector9 gravityError = gravityFactor.evaluateError(
        kErrorStateI, kErrorStateJ, kErrorBias, Point3(params->n_gravity));
    EXPECT(assert_equal(expectedError(pim, mode), gravityError, 1e-12));
    EXPECT(assert_equal(
        gravityError,
        splitGravityFactor.evaluateError(
            kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
            kErrorStateJ.velocity(), kErrorBias, Point3(params->n_gravity)),
        1e-12));
  }
}

// Checks gravity-aware Combined factors for every Combined backend.
TEST_PIM_WITH_COMBINED_BACKEND(ImuFactorErrorMode,
                               CombinedGravityFactorSelection) {
  const auto params = makeCombinedParams();
  const CombinedPIM pim =
      integrateIdealCombinedMeasurements<CombinedPIM>(params);
  const imuBias::ConstantBias biasJ(Vector3(-0.02, 0.01, 0.04),
                                    Vector3(0.002, -0.003, 0.001));
  const CombinedImuFactorWithGravityT<CombinedPIM, Point3> gravityFactor(
      X(1), V(1), X(2), V(2), B(1), B(2), G(1), pim);

  const Vector15 defaultError = gravityFactor.evaluateError(
      kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
      kErrorStateJ.velocity(), kErrorBias, biasJ, Point3(params->n_gravity));
  EXPECT(assert_equal(expectedError(pim, ImuFactorErrorMode::Logmap),
                      Vector9(defaultError.head<9>()), 1e-12));

  const ImuFactorErrorMode modes[] = {ImuFactorErrorMode::Legacy,
                                      ImuFactorErrorMode::ComponentWise,
                                      ImuFactorErrorMode::Logmap};
  for (const ImuFactorErrorMode mode : modes) {
    params->setImuFactorErrorMode(mode);
    const Vector15 actual = gravityFactor.evaluateError(
        kErrorStateI.pose(), kErrorStateI.velocity(), kErrorStateJ.pose(),
        kErrorStateJ.velocity(), kErrorBias, biasJ, Point3(params->n_gravity));
    EXPECT(assert_equal(expectedError(pim, mode), Vector9(actual.head<9>()),
                        1e-12));
  }
}

// Checks optimized propagation against the original dense equation.
TEST_PIM(ImuFactorCovariance, DensePropagationRegression) {
  EXPECT(matchesDensePropagation<PIM>(false));
  EXPECT(matchesDensePropagation<PIM>(true));
}

// Checks optimized Combined propagation against the original dense equation.
TEST_PIM_WITH_COMBINED_BACKEND(ImuFactorCovariance,
                               DenseCombinedPropagationRegression) {
  EXPECT(matchesDenseCombinedPropagation<CombinedPIM>(false));
  EXPECT(matchesDenseCombinedPropagation<CombinedPIM>(true));
}

template <class CombinedPIM>
Matrix15 combinedResidualChartJacobian(const CombinedPIM& pim) {
  Matrix15 jacobian = Matrix15::Identity();
  jacobian.template topLeftCorner<9, 9>() = residualChartJacobian(pim);
  jacobian.template bottomRightCorner<6, 6>() = -I_6x6;
  return jacobian;
}

template <class CombinedPIM>
Matrix15 theoreticalCombinedResidualCovariance(const CombinedPIM& pim) {
  const Matrix15 jacobian = combinedResidualChartJacobian(pim);
  return jacobian * pim.preintMeasCov() * jacobian.transpose();
}

template <class PIM>
Matrix9 installedFactorCovariance(const PIM& pim) {
  const ImuFactor2T<PIM> factor(X(1), X(2), B(1), pim);
  const auto gaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(factor.noiseModel());
  if (!gaussian) {
    throw std::runtime_error("IMU factor does not have a Gaussian noise model");
  }
  return gaussian->covariance();
}

template <class CombinedPIM>
Matrix15 installedCombinedFactorCovariance(const CombinedPIM& pim) {
  const CombinedImuFactorT<CombinedPIM> factor(X(1), V(1), X(2), V(2), B(1),
                                               B(2), pim);
  const auto gaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(factor.noiseModel());
  if (!gaussian) {
    throw std::runtime_error(
        "Combined IMU factor does not have a Gaussian noise model");
  }
  return gaussian->covariance();
}

template <class CombinedPIM>
Matrix15 installedCombinedGravityFactorCovariance(const CombinedPIM& pim) {
  const CombinedImuFactorWithGravityT<CombinedPIM, Point3> factor(
      X(1), V(1), X(2), V(2), B(1), B(2), G(1), pim);
  const auto gaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(factor.noiseModel());
  if (!gaussian) {
    throw std::runtime_error(
        "Gravity-aware combined IMU factor does not have a Gaussian noise "
        "model");
  }
  return gaussian->covariance();
}

template <class ActualDerived, class ExpectedDerived>
double relativeCovarianceError(
    const Eigen::MatrixBase<ActualDerived>& actual,
    const Eigen::MatrixBase<ExpectedDerived>& expected) {
  const double scale =
      std::max(expected.norm(), std::numeric_limits<double>::epsilon());
  return (actual - expected).norm() / scale;
}

template <class ActualDerived, class ExpectedDerived, class PIM>
void expectRelativeCovarianceErrorBelow(
    const Eigen::MatrixBase<ActualDerived>& actual,
    const Eigen::MatrixBase<ExpectedDerived>& expected, const PIM& pim,
    double tolerance, const char* comparison, TestResult& result,
    const std::string& testName, const char* filename, long lineNumber) {
  const double error = relativeCovarianceError(actual, expected);
  if (error < tolerance) return;

  std::ostringstream message;
  message << comparison << " for " << demangle(typeid(pim).name())
          << ": relative error " << error << " >= " << tolerance;
  result.addFailure(Failure(testName, filename, lineNumber, message.str()));
}

template <class PIM>
Matrix9 monteCarloResidualCovariance(
    const std::shared_ptr<PreintegrationParams>& params,
    const NavState& state_i, const NavState& state_j) {
  std::mt19937_64 randomGenerator(kRandomSeed);
  Sampler accelerometerSampler(kAccelerometerSigmas / std::sqrt(kDt),
                               randomGenerator);
  Sampler gyroscopeSampler(kGyroscopeSigmas / std::sqrt(kDt), randomGenerator);

  const imuBias::ConstantBias bias;
  Vector9 mean = Z_9x1;
  Matrix9 sumOfProducts = Z_9x9;

  for (size_t sampleIndex = 1; sampleIndex <= kMonteCarloSamples;
       ++sampleIndex) {
    PIM sampledPim(params, bias);
    for (size_t i = 0; i < kMeasurementCount; ++i) {
      const Vector3 sampledAcceleration =
          kMeasuredAcceleration + accelerometerSampler.sample();
      const Vector3 sampledAngularVelocity =
          kMeasuredAngularVelocity + gyroscopeSampler.sample();
      sampledPim.integrateMeasurement(sampledAcceleration,
                                      sampledAngularVelocity, kDt);
    }

    const ImuFactor2T<PIM> sampledFactor(X(1), X(2), B(1), sampledPim);
    const Vector9 residual =
        sampledFactor.evaluateError(state_i, state_j, bias, {}, {}, {});

    const Vector9 differenceFromOldMean = residual - mean;
    mean += differenceFromOldMean / static_cast<double>(sampleIndex);
    const Vector9 differenceFromNewMean = residual - mean;
    sumOfProducts.noalias() +=
        differenceFromOldMean * differenceFromNewMean.transpose();
  }

  const Matrix9 covariance =
      sumOfProducts / static_cast<double>(kMonteCarloSamples - 1);
  return 0.5 * (covariance + covariance.transpose());
}

template <class CombinedPIM>
Matrix15 monteCarloCombinedResidualCovariance(
    const std::shared_ptr<PreintegrationCombinedParams>& params,
    const NavState& state_i, const NavState& state_j) {
  std::mt19937_64 randomGenerator(kRandomSeed);
  Sampler accelerometerSampler(kAccelerometerSigmas / std::sqrt(kDt),
                               randomGenerator);
  Sampler gyroscopeSampler(kGyroscopeSigmas / std::sqrt(kDt), randomGenerator);
  Sampler accelerometerBiasSampler(
      kAccelerometerBiasRandomWalkSigmas * std::sqrt(kDt), randomGenerator);
  Sampler gyroscopeBiasSampler(kGyroscopeBiasRandomWalkSigmas * std::sqrt(kDt),
                               randomGenerator);

  const imuBias::ConstantBias bias_i;
  Vector15 mean = Vector15::Zero();
  Matrix15 sumOfProducts = Matrix15::Zero();

  for (size_t sampleIndex = 1; sampleIndex <= kMonteCarloSamples;
       ++sampleIndex) {
    CombinedPIM sampledPim(params, bias_i);
    Vector3 accelerometerBias = Z_3x1;
    Vector3 gyroscopeBias = Z_3x1;

    for (size_t i = 0; i < kMeasurementCount; ++i) {
      const Vector3 sampledAcceleration = kMeasuredAcceleration +
                                          accelerometerBias +
                                          accelerometerSampler.sample();
      const Vector3 sampledAngularVelocity =
          kMeasuredAngularVelocity + gyroscopeBias + gyroscopeSampler.sample();
      sampledPim.integrateMeasurement(sampledAcceleration,
                                      sampledAngularVelocity, kDt);

      // The propagation first applies the current bias and then adds the
      // random-walk increment for the next measurement.
      accelerometerBias += accelerometerBiasSampler.sample();
      gyroscopeBias += gyroscopeBiasSampler.sample();
    }

    const imuBias::ConstantBias bias_j(accelerometerBias, gyroscopeBias);
    const CombinedImuFactorT<CombinedPIM> sampledFactor(X(1), V(1), X(2), V(2),
                                                        B(1), B(2), sampledPim);
    const Vector15 residual = sampledFactor.evaluateError(
        state_i.pose(), state_i.v(), state_j.pose(), state_j.v(), bias_i,
        bias_j, {}, {}, {}, {}, {}, {});

    const Vector15 differenceFromOldMean = residual - mean;
    mean += differenceFromOldMean / static_cast<double>(sampleIndex);
    const Vector15 differenceFromNewMean = residual - mean;
    sumOfProducts.noalias() +=
        differenceFromOldMean * differenceFromNewMean.transpose();
  }

  const Matrix15 covariance =
      sumOfProducts / static_cast<double>(kMonteCarloSamples - 1);
  return 0.5 * (covariance + covariance.transpose());
}

// Checks the covariance installed on each factor against its residual chart.
TEST_PIM(ImuFactorCovariance, TheoreticalResidualChart) {
  const auto params = makeParams();
  const PIM pim = integrateIdealMeasurements<PIM>(params);
  const Matrix9 expected = theoreticalResidualCovariance(pim);
  const Matrix9 converted = pim.residualCovariance();
  const Matrix9 installed = installedFactorCovariance(pim);

  expectRelativeCovarianceErrorBelow(
      converted, expected, pim, 1e-10,
      "PIM residual covariance vs theoretical residual covariance", result_,
      name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      installed, expected, pim, 1e-10,
      "installed factor covariance vs theoretical residual covariance", result_,
      name_, __FILE__, __LINE__);
}

// Checks the installed covariance against sampled nonlinear factor residuals.
TEST_PIM(ImuFactorCovariance, MonteCarloResidualChart) {
  const auto params = makeParams();
  const PIM idealPim = integrateIdealMeasurements<PIM>(params);
  const imuBias::ConstantBias bias;
  const NavState state_i;
  const NavState state_j = idealPim.predict(state_i, bias);

  const Matrix9 empirical =
      monteCarloResidualCovariance<PIM>(params, state_i, state_j);
  const Matrix9 theoretical = theoreticalResidualCovariance(idealPim);
  const Matrix9 installed = installedFactorCovariance(idealPim);

  expectRelativeCovarianceErrorBelow(
      empirical, theoretical, idealPim, kMonteCarloTolerance,
      "Monte Carlo covariance vs theoretical residual covariance", result_,
      name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      empirical, installed, idealPim, kMonteCarloTolerance,
      "Monte Carlo covariance vs installed factor covariance", result_, name_,
      __FILE__, __LINE__);
}

// Checks the full combined covariance, including state-bias cross blocks.
TEST_PIM(ImuFactorCovariance, CombinedTheoreticalResidualChart) {
  const auto params = makeCombinedParams();
  const CombinedPIM pim =
      integrateIdealCombinedMeasurements<CombinedPIM>(params);
  const Matrix15 native = pim.preintMeasCov();
  const double nativeCrossCovarianceNorm =
      native.template block<9, 6>(0, 9).norm();
  EXPECT(nativeCrossCovarianceNorm > 1e-8);

  const Matrix15 expected = theoreticalCombinedResidualCovariance(pim);
  const Matrix15 converted = pim.residualCovariance();
  const Matrix15 installed = installedCombinedFactorCovariance(pim);
  const Matrix15 installedWithGravity =
      installedCombinedGravityFactorCovariance(pim);

  expectRelativeCovarianceErrorBelow(
      converted, expected, pim, 1e-10,
      "combined PIM residual covariance vs theoretical residual covariance",
      result_, name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      installed, expected, pim, 1e-10,
      "installed combined factor covariance vs theoretical residual "
      "covariance",
      result_, name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      installedWithGravity, expected, pim, 1e-10,
      "installed gravity-aware combined factor covariance vs theoretical "
      "residual covariance",
      result_, name_, __FILE__, __LINE__);
}

// Checks full combined-factor covariance against sampled nonlinear residuals.
TEST_PIM(ImuFactorCovariance, CombinedMonteCarloResidualChart) {
  const auto params = makeCombinedParams();
  const CombinedPIM idealPim =
      integrateIdealCombinedMeasurements<CombinedPIM>(params);
  const imuBias::ConstantBias bias;
  const NavState state_i;
  const NavState state_j = idealPim.predict(state_i, bias);

  const Matrix15 empirical = monteCarloCombinedResidualCovariance<CombinedPIM>(
      params, state_i, state_j);
  const Matrix15 theoretical = theoreticalCombinedResidualCovariance(idealPim);
  const Matrix15 installed = installedCombinedFactorCovariance(idealPim);
  const Matrix15 installedWithGravity =
      installedCombinedGravityFactorCovariance(idealPim);
  const Matrix96 empiricalCross = empirical.template block<9, 6>(0, 9);
  const Matrix96 theoreticalCross = theoretical.template block<9, 6>(0, 9);
  const Matrix96 installedCross = installed.template block<9, 6>(0, 9);
  const Matrix96 installedWithGravityCross =
      installedWithGravity.template block<9, 6>(0, 9);

  expectRelativeCovarianceErrorBelow(
      empirical, theoretical, idealPim, kMonteCarloTolerance,
      "Monte Carlo combined covariance vs theoretical residual covariance",
      result_, name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      empiricalCross, theoreticalCross, idealPim, 0.20,
      "Monte Carlo combined state-bias covariance vs theoretical residual "
      "state-bias covariance",
      result_, name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      empirical, installed, idealPim, kMonteCarloTolerance,
      "Monte Carlo combined covariance vs installed factor covariance", result_,
      name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      empiricalCross, installedCross, idealPim, 0.20,
      "Monte Carlo combined state-bias covariance vs installed factor "
      "state-bias covariance",
      result_, name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      empirical, installedWithGravity, idealPim, kMonteCarloTolerance,
      "Monte Carlo combined covariance vs installed gravity-aware factor "
      "covariance",
      result_, name_, __FILE__, __LINE__);
  expectRelativeCovarianceErrorBelow(
      empiricalCross, installedWithGravityCross, idealPim, 0.20,
      "Monte Carlo combined state-bias covariance vs installed gravity-aware "
      "factor state-bias covariance",
      result_, name_, __FILE__, __LINE__);
}

}  // namespace imu_covariance
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
