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
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/CombinedImuFactorWithGravity.h>
#include <gtsam/navigation/ImuFactor.h>

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
  static_assert(std::is_same_v<PreintegrationType, ManifoldPreintegration> ||
                    std::is_same_v<PreintegrationType, TangentPreintegration> ||
                    std::is_same_v<PreintegrationType, LieGroupPreintegration>,
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
