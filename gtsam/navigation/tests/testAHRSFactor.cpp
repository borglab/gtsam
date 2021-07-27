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

#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/debug.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <list>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

Vector3 kZeroOmegaCoriolis(0,0,0);

// Define covariance matrices
double accNoiseVar = 0.01;
const Matrix3 kMeasuredAccCovariance = accNoiseVar * I_3x3;

//******************************************************************************
namespace {
Vector callEvaluateError(const AHRSFactor& factor, const Rot3 rot_i,
    const Rot3 rot_j, const Vector3& bias) {
  return factor.evaluateError(rot_i, rot_j, bias);
}

Rot3 evaluateRotationError(const AHRSFactor& factor, const Rot3 rot_i,
    const Rot3 rot_j, const Vector3& bias) {
  return Rot3::Expmap(factor.evaluateError(rot_i, rot_j, bias).tail(3));
}

AHRSFactor::PreintegratedMeasurements evaluatePreintegratedMeasurements(
    const Vector3& bias, const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs,
    const Vector3& initialRotationRate = Vector3::Zero()) {
  AHRSFactor::PreintegratedMeasurements result(bias, I_3x3);

  list<Vector3>::const_iterator itOmega = measuredOmegas.begin();
  list<double>::const_iterator itDeltaT = deltaTs.begin();
  for (; itOmega != measuredOmegas.end(); ++itOmega, ++itDeltaT) {
    result.integrateMeasurement(*itOmega, *itDeltaT);
  }

  return result;
}

Rot3 evaluatePreintegratedMeasurementsRotation(
    const Vector3& bias, const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs,
    const Vector3& initialRotationRate = Vector3::Zero()) {
  return Rot3(
      evaluatePreintegratedMeasurements(bias, measuredOmegas, deltaTs,
          initialRotationRate).deltaRij());
}

Rot3 evaluateRotation(const Vector3 measuredOmega, const Vector3 biasOmega,
    const double deltaT) {
  return Rot3::Expmap((measuredOmega - biasOmega) * deltaT);
}

Vector3 evaluateLogRotation(const Vector3 thetahat, const Vector3 deltatheta) {
  return Rot3::Logmap(Rot3::Expmap(thetahat).compose(Rot3::Expmap(deltatheta)));
}

}
//******************************************************************************
TEST( AHRSFactor, PreintegratedMeasurements ) {
  // Linearization point
  Vector3 bias(0,0,0); ///< Current estimate of angular rate bias

  // Measurements
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected preintegrated values
  Rot3 expectedDeltaR1 = Rot3::RzRyRx(0.5 * M_PI / 100.0, 0.0, 0.0);
  double expectedDeltaT1(0.5);

  // Actual preintegrated values
  AHRSFactor::PreintegratedMeasurements actual1(bias, Z_3x3);
  actual1.integrateMeasurement(measuredOmega, deltaT);

  EXPECT(assert_equal(expectedDeltaR1, Rot3(actual1.deltaRij()), 1e-6));
  DOUBLES_EQUAL(expectedDeltaT1, actual1.deltaTij(), 1e-6);

  // Integrate again
  Rot3 expectedDeltaR2 = Rot3::RzRyRx(2.0 * 0.5 * M_PI / 100.0, 0.0, 0.0);
  double expectedDeltaT2(1);

  // Actual preintegrated values
  AHRSFactor::PreintegratedMeasurements actual2 = actual1;
  actual2.integrateMeasurement(measuredOmega, deltaT);

  EXPECT(assert_equal(expectedDeltaR2, Rot3(actual2.deltaRij()), 1e-6));
  DOUBLES_EQUAL(expectedDeltaT2, actual2.deltaTij(), 1e-6);
}

//******************************************************************************
TEST( AHRSFactor, PreintegratedAhrsMeasurementsConstructor ) {
  Matrix3 gyroscopeCovariance = Matrix3::Ones()*0.4;
  Vector3 omegaCoriolis(0.1, 0.5, 0.9);
  PreintegratedRotationParams params(gyroscopeCovariance, omegaCoriolis);
  Vector3 bias(1.0,2.0,3.0); ///< Current estimate of angular rate bias
  Rot3 deltaRij(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0));
  double deltaTij = 0.02;
  Matrix3 delRdelBiasOmega = Matrix3::Ones()*0.5;
  Matrix3 preintMeasCov = Matrix3::Ones()*0.2;
  PreintegratedAhrsMeasurements actualPim(
    boost::make_shared<PreintegratedRotationParams>(params),
    bias,
    deltaTij,
    deltaRij,
    delRdelBiasOmega,
    preintMeasCov);
  EXPECT(assert_equal(gyroscopeCovariance,
      actualPim.p().getGyroscopeCovariance(), 1e-6));
  EXPECT(assert_equal(omegaCoriolis,
      actualPim.p().getOmegaCoriolis().get(), 1e-6));
  EXPECT(assert_equal(bias, actualPim.biasHat(), 1e-6));
  DOUBLES_EQUAL(deltaTij, actualPim.deltaTij(), 1e-6);
  EXPECT(assert_equal(deltaRij, Rot3(actualPim.deltaRij()), 1e-6));
  EXPECT(assert_equal(delRdelBiasOmega, actualPim.delRdelBiasOmega(), 1e-6));
  EXPECT(assert_equal(preintMeasCov, actualPim.preintMeasCov(), 1e-6));
}

/* ************************************************************************* */
TEST(AHRSFactor, Error) {
  // Linearization point
  Vector3 bias(0.,0.,0.); // Bias
  Rot3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0));
  Rot3 x2(Rot3::RzRyRx(M_PI / 12.0 + M_PI / 100.0, M_PI / 6.0, M_PI / 4.0));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << M_PI / 100, 0, 0;
  double deltaT = 1.0;
  AHRSFactor::PreintegratedMeasurements pim(bias, Z_3x3);
  pim.integrateMeasurement(measuredOmega, deltaT);

  // Create factor
  AHRSFactor factor(X(1), X(2), B(1), pim, kZeroOmegaCoriolis, boost::none);

  Vector3 errorActual = factor.evaluateError(x1, x2, bias);

  // Expected error
  Vector3 errorExpected(3);
  errorExpected << 0, 0, 0;
  EXPECT(assert_equal(Vector(errorExpected), Vector(errorActual), 1e-6));

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Vector3, Rot3>(
      boost::bind(&callEvaluateError, factor, _1, x2, bias), x1);
  Matrix H2e = numericalDerivative11<Vector3, Rot3>(
      boost::bind(&callEvaluateError, factor, x1, _1, bias), x2);
  Matrix H3e = numericalDerivative11<Vector3, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, x2, _1), bias);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&evaluateRotationError, factor, _1, x2, bias), x1);
  Matrix RH2e = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&evaluateRotationError, factor, x1, _1, bias), x2);

  // Actual Jacobians
  Matrix H1a, H2a, H3a;
  (void) factor.evaluateError(x1, x2, bias, H1a, H2a, H3a);

  // rotations
  EXPECT(assert_equal(RH1e, H1a, 1e-5));
  // 1e-5 needs to be added only when using quaternions for rotations

  EXPECT(assert_equal(H2e, H2a, 1e-5));

  // rotations
  EXPECT(assert_equal(RH2e, H2a, 1e-5));
  // 1e-5 needs to be added only when using quaternions for rotations

  EXPECT(assert_equal(H3e, H3a, 1e-5));
  // 1e-5 needs to be added only when using quaternions for rotations
}

/* ************************************************************************* */
TEST(AHRSFactor, ErrorWithBiases) {
  // Linearization point

  Vector3 bias(0, 0, 0.3);
  Rot3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)));
  Rot3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  double deltaT = 1.0;

  AHRSFactor::PreintegratedMeasurements pim(Vector3(0,0,0),
      Z_3x3);
  pim.integrateMeasurement(measuredOmega, deltaT);

  // Create factor
  AHRSFactor factor(X(1), X(2), B(1), pim, kZeroOmegaCoriolis);

  Vector errorActual = factor.evaluateError(x1, x2, bias);

  // Expected error
  Vector errorExpected(3);
  errorExpected << 0, 0, 0;
  EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Vector, Rot3>(
      boost::bind(&callEvaluateError, factor, _1, x2, bias), x1);
  Matrix H2e = numericalDerivative11<Vector, Rot3>(
      boost::bind(&callEvaluateError, factor, x1, _1, bias), x2);
  Matrix H3e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, x2, _1), bias);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&evaluateRotationError, factor, _1, x2, bias), x1);
  Matrix RH2e = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&evaluateRotationError, factor, x1, _1, bias), x2);
  Matrix RH3e = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&evaluateRotationError, factor, x1, x2, _1), bias);

  // Actual Jacobians
  Matrix H1a, H2a, H3a;
  (void) factor.evaluateError(x1, x2, bias, H1a, H2a, H3a);

  EXPECT(assert_equal(H1e, H1a));
  EXPECT(assert_equal(H2e, H2a));
  EXPECT(assert_equal(H3e, H3a));
}

//******************************************************************************
TEST( AHRSFactor, PartialDerivativeExpmap ) {
  // Linearization point
  Vector3 biasOmega(0,0,0);

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0.1, 0, 0;
  double deltaT = 0.5;

  // Compute numerical derivatives
  Matrix expectedDelRdelBiasOmega = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&evaluateRotation, measuredOmega, _1, deltaT), biasOmega);

  const Matrix3 Jr = Rot3::ExpmapDerivative(
      (measuredOmega - biasOmega) * deltaT);

  Matrix3 actualdelRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelRdelBiasOmega, actualdelRdelBiasOmega, 1e-3));
  // 1e-3 needs to be added only when using quaternions for rotations

}

//******************************************************************************
TEST( AHRSFactor, PartialDerivativeLogmap ) {
  // Linearization point
  Vector3 thetahat;
  thetahat << 0.1, 0.1, 0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 deltatheta;
  deltatheta << 0, 0, 0;

  // Compute numerical derivatives
  Matrix expectedDelFdeltheta = numericalDerivative11<Vector3, Vector3>(
      boost::bind(&evaluateLogRotation, thetahat, _1), deltatheta);

  const Vector3 x = thetahat; // parametrization of so(3)
  const Matrix3 X = skewSymmetric(x); // element of Lie algebra so(3): X = x^
  double normx = x.norm();
  const Matrix3 actualDelFdeltheta = I_3x3 + 0.5 * X
      + (1 / (normx * normx) - (1 + cos(normx)) / (2 * normx * sin(normx))) * X
          * X;

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelFdeltheta, actualDelFdeltheta));

}

//******************************************************************************
TEST( AHRSFactor, fistOrderExponential ) {
  // Linearization point
  Vector3 biasOmega(0,0,0);

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0.1, 0, 0;
  double deltaT = 1.0;

  // change w.r.t. linearization point
  double alpha = 0.0;
  Vector3 deltabiasOmega;
  deltabiasOmega << alpha, alpha, alpha;

  const Matrix3 Jr = Rot3::ExpmapDerivative(
      (measuredOmega - biasOmega) * deltaT);

  Matrix3 delRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  const Matrix expectedRot = Rot3::Expmap(
      (measuredOmega - biasOmega - deltabiasOmega) * deltaT).matrix();

  const Matrix3 hatRot =
      Rot3::Expmap((measuredOmega - biasOmega) * deltaT).matrix();
  const Matrix3 actualRot = hatRot
      * Rot3::Expmap(delRdelBiasOmega * deltabiasOmega).matrix();

  // Compare Jacobians
  EXPECT(assert_equal(expectedRot, actualRot));
}

//******************************************************************************
TEST( AHRSFactor, FirstOrderPreIntegratedMeasurements ) {
  // Linearization point
  Vector3 bias = Vector3::Zero(); ///< Current estimate of rotation rate bias

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
  AHRSFactor::PreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(bias, measuredOmegas, deltaTs,
          Vector3(M_PI / 100.0, 0.0, 0.0));

  // Compute numerical derivatives
  Matrix expectedDelRdelBias =
      numericalDerivative11<Rot3, Vector3>(
          boost::bind(&evaluatePreintegratedMeasurementsRotation, _1,
              measuredOmegas, deltaTs, Vector3(M_PI / 100.0, 0.0, 0.0)), bias);
  Matrix expectedDelRdelBiasOmega = expectedDelRdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(
      assert_equal(expectedDelRdelBiasOmega, preintegrated.delRdelBiasOmega(), 1e-3));
  // 1e-3 needs to be added only when using quaternions for rotations
}

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

//******************************************************************************
TEST( AHRSFactor, ErrorWithBiasesAndSensorBodyDisplacement ) {

  Vector3 bias(0, 0, 0.3);
  Rot3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)));
  Rot3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)));

  // Measurements
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0.1, 0.1;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  double deltaT = 1.0;

  const Pose3 body_P_sensor(Rot3::Expmap(Vector3(0, 0.10, 0.10)),
      Point3(1, 0, 0));

  AHRSFactor::PreintegratedMeasurements pim(Vector3::Zero(), kMeasuredAccCovariance);

  pim.integrateMeasurement(measuredOmega, deltaT);

  // Check preintegrated covariance
  EXPECT(assert_equal(kMeasuredAccCovariance, pim.preintMeasCov()));

  // Create factor
  AHRSFactor factor(X(1), X(2), B(1), pim, omegaCoriolis);

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Vector, Rot3>(
      boost::bind(&callEvaluateError, factor, _1, x2, bias), x1);
  Matrix H2e = numericalDerivative11<Vector, Rot3>(
      boost::bind(&callEvaluateError, factor, x1, _1, bias), x2);
  Matrix H3e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, x2, _1), bias);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&evaluateRotationError, factor, _1, x2, bias), x1);
  Matrix RH2e = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&evaluateRotationError, factor, x1, _1, bias), x2);
  Matrix RH3e = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&evaluateRotationError, factor, x1, x2, _1), bias);

  // Actual Jacobians
  Matrix H1a, H2a, H3a;
  (void) factor.evaluateError(x1, x2, bias, H1a, H2a, H3a);

  EXPECT(assert_equal(H1e, H1a));
  EXPECT(assert_equal(H2e, H2a));
  EXPECT(assert_equal(H3e, H3a));
}
//******************************************************************************
TEST (AHRSFactor, predictTest) {
  Vector3 bias(0,0,0);

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0;
  double deltaT = 0.2;
  AHRSFactor::PreintegratedMeasurements pim(bias, kMeasuredAccCovariance);
  for (int i = 0; i < 1000; ++i) {
    pim.integrateMeasurement(measuredOmega, deltaT);
  }
  // Check preintegrated covariance
  Matrix expectedMeasCov(3,3);
  expectedMeasCov = 200*kMeasuredAccCovariance;
  EXPECT(assert_equal(expectedMeasCov, pim.preintMeasCov()));

  AHRSFactor factor(X(1), X(2), B(1), pim, kZeroOmegaCoriolis);

  // Predict
  Rot3 x;
  Rot3 expectedRot = Rot3::Ypr(20*M_PI, 0, 0);
  Rot3 actualRot = factor.predict(x, bias, pim, kZeroOmegaCoriolis);
  EXPECT(assert_equal(expectedRot, actualRot, 1e-6));

  // AHRSFactor::PreintegratedMeasurements::predict
  Matrix expectedH = numericalDerivative11<Vector3, Vector3>(
      boost::bind(&AHRSFactor::PreintegratedMeasurements::predict,
          &pim, _1, boost::none), bias);

  // Actual Jacobians
  Matrix H;
  (void) pim.predict(bias,H);
  EXPECT(assert_equal(expectedH, H, 1e-8));
}
//******************************************************************************
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

TEST (AHRSFactor, graphTest) {
  // linearization point
  Rot3 x1(Rot3::RzRyRx(0, 0, 0));
  Rot3 x2(Rot3::RzRyRx(0, M_PI / 4, 0));
  Vector3 bias(0,0,0);

  // PreIntegrator
  Vector3 biasHat(0, 0, 0);
  AHRSFactor::PreintegratedMeasurements pim(biasHat, kMeasuredAccCovariance);

  // Pre-integrate measurements
  Vector3 measuredOmega(0, M_PI / 20, 0);
  double deltaT = 1;

  // Create Factor
  noiseModel::Base::shared_ptr model = //
      noiseModel::Gaussian::Covariance(pim.preintMeasCov());
  NonlinearFactorGraph graph;
  Values values;
  for (size_t i = 0; i < 5; ++i) {
    pim.integrateMeasurement(measuredOmega, deltaT);
  }

  // pim.print("Pre integrated measurementes");
  AHRSFactor factor(X(1), X(2), B(1), pim, kZeroOmegaCoriolis);
  values.insert(X(1), x1);
  values.insert(X(2), x2);
  values.insert(B(1), bias);
  graph.push_back(factor);
  LevenbergMarquardtOptimizer optimizer(graph, values);
  Values result = optimizer.optimize();
  Rot3 expectedRot(Rot3::RzRyRx(0, M_PI / 4, 0));
  EXPECT(assert_equal(expectedRot, result.at<Rot3>(X(2))));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
