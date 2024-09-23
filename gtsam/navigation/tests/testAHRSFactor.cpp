/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testAHRSFactor.cpp
 * @brief  Unit test for AHRSFactor
 * @author Krunal Chande
 * @author Luca Carlone
 * @author Frank Dellaert
 * @author Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <list>
#include <memory>
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::B;
using symbol_shorthand::R;

Vector3 kZeroOmegaCoriolis(0, 0, 0);

// Define covariance matrices
double gyroNoiseVar = 0.01;
const Matrix3 kMeasuredOmegaCovariance = gyroNoiseVar * I_3x3;

//******************************************************************************
namespace {
PreintegratedAhrsMeasurements integrateMeasurements(
    const Vector3& biasHat, const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs) {
  PreintegratedAhrsMeasurements result(biasHat, I_3x3);

  list<Vector3>::const_iterator itOmega = measuredOmegas.begin();
  list<double>::const_iterator itDeltaT = deltaTs.begin();
  for (; itOmega != measuredOmegas.end(); ++itOmega, ++itDeltaT) {
    result.integrateMeasurement(*itOmega, *itDeltaT);
  }

  return result;
}
}  // namespace

//******************************************************************************
TEST(AHRSFactor, PreintegratedAhrsMeasurements) {
  // Linearization point
  Vector3 biasHat(0, 0, 0);  ///< Current estimate of angular rate bias

  // Measurements
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected preintegrated values
  Rot3 expectedDeltaR1 = Rot3::Roll(0.5 * M_PI / 100.0);

  // Actual preintegrated values
  PreintegratedAhrsMeasurements actual1(biasHat, kMeasuredOmegaCovariance);
  actual1.integrateMeasurement(measuredOmega, deltaT);

  EXPECT(assert_equal(expectedDeltaR1, Rot3(actual1.deltaRij()), 1e-6));
  DOUBLES_EQUAL(deltaT, actual1.deltaTij(), 1e-6);

  // Check the covariance
  Matrix3 expectedMeasCov = kMeasuredOmegaCovariance * deltaT;
  EXPECT(assert_equal(expectedMeasCov, actual1.preintMeasCov(), 1e-6));

  // Integrate again
  Rot3 expectedDeltaR2 = Rot3::Roll(2.0 * 0.5 * M_PI / 100.0);

  // Actual preintegrated values
  PreintegratedAhrsMeasurements actual2 = actual1;
  actual2.integrateMeasurement(measuredOmega, deltaT);

  EXPECT(assert_equal(expectedDeltaR2, Rot3(actual2.deltaRij()), 1e-6));
  DOUBLES_EQUAL(deltaT * 2, actual2.deltaTij(), 1e-6);
}

//******************************************************************************
TEST(AHRSFactor, PreintegratedAhrsMeasurementsConstructor) {
  Matrix3 gyroscopeCovariance = I_3x3 * 0.4;
  Vector3 omegaCoriolis(0.1, 0.5, 0.9);
  PreintegratedRotationParams params(gyroscopeCovariance, omegaCoriolis);
  Vector3 bias(1.0, 2.0, 3.0);  ///< Current estimate of angular rate bias
  Rot3 deltaRij(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0));
  double deltaTij = 0.02;
  Matrix3 delRdelBiasOmega = I_3x3 * 0.5;
  Matrix3 preintMeasCov = I_3x3 * 0.2;
  PreintegratedAhrsMeasurements actualPim(
      std::make_shared<PreintegratedRotationParams>(params), bias, deltaTij,
      deltaRij, delRdelBiasOmega, preintMeasCov);
  EXPECT(assert_equal(gyroscopeCovariance,
                      actualPim.p().getGyroscopeCovariance(), 1e-6));
  EXPECT(
      assert_equal(omegaCoriolis, *(actualPim.p().getOmegaCoriolis()), 1e-6));
  EXPECT(assert_equal(bias, actualPim.biasHat(), 1e-6));
  DOUBLES_EQUAL(deltaTij, actualPim.deltaTij(), 1e-6);
  EXPECT(assert_equal(deltaRij, Rot3(actualPim.deltaRij()), 1e-6));
  EXPECT(assert_equal(delRdelBiasOmega, actualPim.delRdelBiasOmega(), 1e-6));
  EXPECT(assert_equal(preintMeasCov, actualPim.preintMeasCov(), 1e-6));
}

/* ************************************************************************* */
TEST(AHRSFactor, Error) {
  // Linearization point
  Vector3 bias(0., 0., 0.);  // Bias
  Rot3 Ri(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0));
  Rot3 Rj(Rot3::RzRyRx(M_PI / 12.0 + M_PI / 100.0, M_PI / 6.0, M_PI / 4.0));

  // Measurements
  Vector3 measuredOmega(M_PI / 100, 0, 0);
  double deltaT = 1.0;
  PreintegratedAhrsMeasurements pim(bias, kMeasuredOmegaCovariance);
  pim.integrateMeasurement(measuredOmega, deltaT);

  // Create factor
  AHRSFactor factor(R(1), R(2), B(1), pim, kZeroOmegaCoriolis, {});

  // Check value
  Vector3 errorActual = factor.evaluateError(Ri, Rj, bias);
  Vector3 errorExpected(0, 0, 0);
  EXPECT(assert_equal(Vector(errorExpected), Vector(errorActual), 1e-6));

  // Check Derivatives
  Values values;
  values.insert(R(1), Ri);
  values.insert(R(2), Rj);
  values.insert(B(1), bias);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

/* ************************************************************************* */
TEST(AHRSFactor, ErrorWithBiases) {
  // Linearization point
  Vector3 bias(0, 0, 0.3);
  Rot3 Ri(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)));
  Rot3 Rj(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)));

  // Measurements
  Vector3 measuredOmega(0, 0, M_PI / 10.0 + 0.3);
  double deltaT = 1.0;
  PreintegratedAhrsMeasurements pim(Vector3(0, 0, 0), kMeasuredOmegaCovariance);
  pim.integrateMeasurement(measuredOmega, deltaT);

  // Create factor
  AHRSFactor factor(R(1), R(2), B(1), pim, kZeroOmegaCoriolis);

  // Check value
  Vector3 errorExpected(0, 0, 0);
  Vector3 errorActual = factor.evaluateError(Ri, Rj, bias);
  EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Check Derivatives
  Values values;
  values.insert(R(1), Ri);
  values.insert(R(2), Rj);
  values.insert(B(1), bias);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//******************************************************************************
TEST(AHRSFactor, PartialDerivativeExpmap) {
  // Linearization point
  Vector3 biasOmega(0, 0, 0);

  // Measurements
  Vector3 measuredOmega(0.1, 0, 0);
  double deltaT = 0.5;

  auto f = [&](const Vector3& biasOmega) {
    return Rot3::Expmap((measuredOmega - biasOmega) * deltaT);
  };

  // Compute numerical derivatives
  Matrix expectedH = numericalDerivative11<Rot3, Vector3>(f, biasOmega);

  const Matrix3 Jr =
      Rot3::ExpmapDerivative((measuredOmega - biasOmega) * deltaT);

  Matrix3 actualH = -Jr * deltaT;  // the delta bias appears with the minus sign

  // Compare Jacobians
  EXPECT(assert_equal(expectedH, actualH, 1e-3));
  // 1e-3 needs to be added only when using quaternions for rotations
}

//******************************************************************************
TEST(AHRSFactor, PartialDerivativeLogmap) {
  // Linearization point
  Vector3 thetaHat(0.1, 0.1, 0);  ///< Current estimate of rotation rate bias

  auto f = [thetaHat](const Vector3 deltaTheta) {
    return Rot3::Logmap(
        Rot3::Expmap(thetaHat).compose(Rot3::Expmap(deltaTheta)));
  };

  // Compute numerical derivatives
  Vector3 deltaTheta(0, 0, 0);
  Matrix expectedH = numericalDerivative11<Vector3, Vector3>(f, deltaTheta);

  const Vector3 x = thetaHat;          // parametrization of so(3)
  const Matrix3 X = skewSymmetric(x);  // element of Lie algebra so(3): X = x^
  double norm = x.norm();
  const Matrix3 actualH =
      I_3x3 + 0.5 * X +
      (1 / (norm * norm) - (1 + cos(norm)) / (2 * norm * sin(norm))) * X * X;

  // Compare Jacobians
  EXPECT(assert_equal(expectedH, actualH));
}

//******************************************************************************
TEST(AHRSFactor, fistOrderExponential) {
  // Linearization point
  Vector3 biasOmega(0, 0, 0);

  // Measurements
  Vector3 measuredOmega(0.1, 0, 0);
  double deltaT = 1.0;

  // change w.r.t. linearization point
  double alpha = 0.0;
  Vector3 deltaBiasOmega(alpha, alpha, alpha);

  const Matrix3 Jr =
      Rot3::ExpmapDerivative((measuredOmega - biasOmega) * deltaT);

  Matrix3 delRdelBiasOmega =
      -Jr * deltaT;  // the delta bias appears with the minus sign

  const Matrix expectedRot =
      Rot3::Expmap((measuredOmega - biasOmega - deltaBiasOmega) * deltaT)
          .matrix();

  const Matrix3 hatRot =
      Rot3::Expmap((measuredOmega - biasOmega) * deltaT).matrix();
  const Matrix3 actualRot =
      hatRot * Rot3::Expmap(delRdelBiasOmega * deltaBiasOmega).matrix();

  // Compare Jacobians
  EXPECT(assert_equal(expectedRot, actualRot));
}

//******************************************************************************
TEST(AHRSFactor, FirstOrderPreIntegratedMeasurements) {
  // Linearization point
  Vector3 bias = Vector3::Zero();  ///< Current estimate of rotation rate bias

  Pose3 body_P_sensor(Rot3::Expmap(Vector3(0, 0.1, 0.1)), Point3(1, 0, 1));

  // Measurements
  list<Vector3> measuredOmegas;
  list<double> deltaTs;
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  for (int i = 1; i < 100; i++) {
    measuredOmegas.push_back(
        Vector3(M_PI / 100.0, M_PI / 300.0, 2 * M_PI / 100.0));
    deltaTs.push_back(0.01);
  }

  // Actual preintegrated values
  PreintegratedAhrsMeasurements preintegrated =
      integrateMeasurements(bias, measuredOmegas, deltaTs);

  auto f = [&](const Vector3& bias) {
    return integrateMeasurements(bias, measuredOmegas, deltaTs).deltaRij();
  };

  // Compute numerical derivatives
  Matrix expectedDelRdelBias = numericalDerivative11<Rot3, Vector3>(f, bias);
  Matrix expectedDelRdelBiasOmega = expectedDelRdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelRdelBiasOmega,
                      preintegrated.delRdelBiasOmega(), 1e-3));
  // 1e-3 needs to be added only when using quaternions for rotations
}

//******************************************************************************
TEST(AHRSFactor, ErrorWithBiasesAndSensorBodyDisplacement) {
  Vector3 bias(0, 0, 0.3);
  Rot3 Ri(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)));
  Rot3 Rj(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)));

  // Measurements
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0.1, 0.1;
  Vector3 measuredOmega(0, 0, M_PI / 10.0 + 0.3);
  double deltaT = 1.0;

  auto p = std::make_shared<PreintegratedAhrsMeasurements::Params>();
  p->gyroscopeCovariance = kMeasuredOmegaCovariance;
  p->body_P_sensor = Pose3(Rot3::Expmap(Vector3(1, 2, 3)), Point3(1, 0, 0));
  PreintegratedAhrsMeasurements pim(p, Vector3::Zero());

  pim.integrateMeasurement(measuredOmega, deltaT);

  // Check preintegrated covariance
  EXPECT(assert_equal(kMeasuredOmegaCovariance, pim.preintMeasCov()));

  // Create factor
  AHRSFactor factor(R(1), R(2), B(1), pim, omegaCoriolis);

  // Check Derivatives
  Values values;
  values.insert(R(1), Ri);
  values.insert(R(2), Rj);
  values.insert(B(1), bias);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

//******************************************************************************
TEST(AHRSFactor, predictTest) {
  Vector3 bias(0, 0, 0);

  // Measurements
  Vector3 measuredOmega(0, 0, M_PI / 10.0);
  double deltaT = 0.2;
  PreintegratedAhrsMeasurements pim(bias, kMeasuredOmegaCovariance);
  for (int i = 0; i < 1000; ++i) {
    pim.integrateMeasurement(measuredOmega, deltaT);
  }
  // Check preintegrated covariance
  Matrix expectedMeasCov(3, 3);
  expectedMeasCov = 200 * kMeasuredOmegaCovariance;
  EXPECT(assert_equal(expectedMeasCov, pim.preintMeasCov()));

  AHRSFactor factor(R(1), R(2), B(1), pim, kZeroOmegaCoriolis);

  // Predict
  Rot3 x;
  Rot3 expectedRot = Rot3::Ypr(20 * M_PI, 0, 0);
  Rot3 actualRot = factor.predict(x, bias, pim, kZeroOmegaCoriolis);
  EXPECT(assert_equal(expectedRot, actualRot, 1e-6));

  // PreintegratedAhrsMeasurements::predict
  Matrix expectedH = numericalDerivative11<Vector3, Vector3>(
      [&pim](const Vector3& b) { return pim.predict(b, {}); }, bias);

  // Actual Jacobians
  Matrix H;
  (void)pim.predict(bias, H);
  EXPECT(assert_equal(expectedH, H, 1e-8));
}
//******************************************************************************
TEST(AHRSFactor, graphTest) {
  // linearization point
  Rot3 Ri(Rot3::RzRyRx(0, 0, 0));
  Rot3 Rj(Rot3::RzRyRx(0, M_PI / 4, 0));
  Vector3 bias(0, 0, 0);

  // PreIntegrator
  Vector3 biasHat(0, 0, 0);
  PreintegratedAhrsMeasurements pim(biasHat, kMeasuredOmegaCovariance);

  // Pre-integrate measurements
  Vector3 measuredOmega(0, M_PI / 20, 0);
  double deltaT = 1;

  // Create Factor
  noiseModel::Base::shared_ptr model =  //
      noiseModel::Gaussian::Covariance(pim.preintMeasCov());
  NonlinearFactorGraph graph;
  Values values;
  for (size_t i = 0; i < 5; ++i) {
    pim.integrateMeasurement(measuredOmega, deltaT);
  }

  // pim.print("Pre integrated measurements");
  AHRSFactor factor(R(1), R(2), B(1), pim, kZeroOmegaCoriolis);
  values.insert(R(1), Ri);
  values.insert(R(2), Rj);
  values.insert(B(1), bias);
  graph.push_back(factor);
  LevenbergMarquardtOptimizer optimizer(graph, values);
  Values result = optimizer.optimize();
  Rot3 expectedRot(Rot3::RzRyRx(0, M_PI / 4, 0));
  EXPECT(assert_equal(expectedRot, result.at<Rot3>(R(2))));
}

/* ************************************************************************* */
TEST(AHRSFactor, bodyPSensorWithBias) {
  using noiseModel::Diagonal;

  int numRotations = 10;
  const Vector3 noiseBetweenBiasSigma(3.0e-6, 3.0e-6, 3.0e-6);
  SharedDiagonal biasNoiseModel = Diagonal::Sigmas(noiseBetweenBiasSigma);

  // Measurements in the sensor frame:
  const double omega = 0.1;
  const Vector3 realOmega(omega, 0, 0);
  const Vector3 realBias(1, 2, 3);  // large !
  const Vector3 measuredOmega = realOmega + realBias;

  auto p = std::make_shared<PreintegratedAhrsMeasurements::Params>();
  p->body_P_sensor = Pose3(Rot3::Yaw(M_PI_2), Point3(0, 0, 0));
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  double deltaT = 0.005;

  // Specify noise values on priors
  const Vector3 priorNoisePoseSigmas(0.001, 0.001, 0.001);
  const Vector3 priorNoiseBiasSigmas(0.5e-1, 0.5e-1, 0.5e-1);
  SharedDiagonal priorNoisePose = Diagonal::Sigmas(priorNoisePoseSigmas);
  SharedDiagonal priorNoiseBias = Diagonal::Sigmas(priorNoiseBiasSigmas);

  // Create a factor graph with priors on initial pose, velocity and bias
  NonlinearFactorGraph graph;
  Values values;

  graph.addPrior(R(0), Rot3(), priorNoisePose);
  values.insert(R(0), Rot3());

  // The key to this test is that we specify the bias, in the sensor frame, as
  // known a priori. We also create factors below that encode our assumption
  // that this bias is constant over time In theory, after optimization, we
  // should recover that same bias estimate
  graph.addPrior(B(0), realBias, priorNoiseBias);
  values.insert(B(0), realBias);

  // Now add IMU factors and bias noise models
  const Vector3 zeroBias(0, 0, 0);
  for (int i = 1; i < numRotations; i++) {
    PreintegratedAhrsMeasurements pim(p, realBias);
    for (int j = 0; j < 200; ++j)
      pim.integrateMeasurement(measuredOmega, deltaT);

    // Create factors
    graph.emplace_shared<AHRSFactor>(R(i - 1), R(i), B(i - 1), pim);
    graph.emplace_shared<BetweenFactor<Vector3> >(B(i - 1), B(i), zeroBias,
                                                  biasNoiseModel);

    values.insert(R(i), Rot3());
    values.insert(B(i), realBias);
  }

  // Finally, optimize, and get bias at last time step
  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  Values result = LevenbergMarquardtOptimizer(graph, values, params).optimize();
  const Vector3 biasActual = result.at<Vector3>(B(numRotations - 1));

  // Bias should be a self-fulfilling prophesy:
  EXPECT(assert_equal(realBias, biasActual, 1e-3));

  // Check that the successive rotations are all `omega` apart:
  for (int i = 0; i < numRotations; i++) {
    Rot3 expectedRot = Rot3::Pitch(omega * i);
    Rot3 actualRot = result.at<Rot3>(R(i));
    EXPECT(assert_equal(expectedRot, actualRot, 1e-3));
  }
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
