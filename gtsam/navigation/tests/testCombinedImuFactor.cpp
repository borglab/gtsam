/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCombinedImuFactor.cpp
 * @brief   Unit test for Lupton-style combined IMU factor
 * @author  Luca Carlone
 * @author  Stephen Williams
 * @author  Richard Roberts
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <list>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

namespace {
/* ************************************************************************* */
// Auxiliary functions to test Jacobians F and G used for
// covariance propagation during preintegration
/* ************************************************************************* */
Vector updatePreintegratedMeasurementsTest(const Vector3 deltaPij_old,
    const Vector3& deltaVij_old, const Rot3& deltaRij_old,
    const imuBias::ConstantBias& bias_old, const Vector3& correctedAcc,
    const Vector3& correctedOmega, const double deltaT,
    const bool use2ndOrderIntegration) {

  Matrix3 dRij = deltaRij_old.matrix();
  Vector3 temp = dRij * (correctedAcc - bias_old.accelerometer()) * deltaT;
  Vector3 deltaPij_new;
  if (!use2ndOrderIntegration) {
    deltaPij_new = deltaPij_old + deltaVij_old * deltaT;
  } else {
    deltaPij_new = deltaPij_old + deltaVij_old * deltaT + 0.5 * temp * deltaT;
  }
  Vector3 deltaVij_new = deltaVij_old + temp;
  Rot3 deltaRij_new = deltaRij_old
      * Rot3::Expmap((correctedOmega - bias_old.gyroscope()) * deltaT);
  Vector3 logDeltaRij_new = Rot3::Logmap(deltaRij_new); // not important any more
  imuBias::ConstantBias bias_new(bias_old);
  Vector result(15);
  result << deltaPij_new, deltaVij_new, logDeltaRij_new, bias_new.vector();
  return result;
}

Rot3 updatePreintegratedMeasurementsRot(const Vector3 deltaPij_old,
    const Vector3& deltaVij_old, const Rot3& deltaRij_old,
    const imuBias::ConstantBias& bias_old, const Vector3& correctedAcc,
    const Vector3& correctedOmega, const double deltaT,
    const bool use2ndOrderIntegration) {

  Vector result = updatePreintegratedMeasurementsTest(deltaPij_old,
      deltaVij_old, deltaRij_old, bias_old, correctedAcc, correctedOmega,
      deltaT, use2ndOrderIntegration);

  return Rot3::Expmap(result.segment<3>(6));
}

// Auxiliary functions to test preintegrated Jacobians
// delPdelBiasAcc_ delPdelBiasOmega_ delVdelBiasAcc_ delVdelBiasOmega_ delRdelBiasOmega_
/* ************************************************************************* */
CombinedImuFactor::CombinedPreintegratedMeasurements evaluatePreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  CombinedImuFactor::CombinedPreintegratedMeasurements result(bias,
      Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity(),
      Matrix3::Identity(), Matrix3::Identity(), Matrix::Identity(6, 6), false);

  list<Vector3>::const_iterator itAcc = measuredAccs.begin();
  list<Vector3>::const_iterator itOmega = measuredOmegas.begin();
  list<double>::const_iterator itDeltaT = deltaTs.begin();
  for (; itAcc != measuredAccs.end(); ++itAcc, ++itOmega, ++itDeltaT) {
    result.integrateMeasurement(*itAcc, *itOmega, *itDeltaT);
  }
  return result;
}

Vector3 evaluatePreintegratedMeasurementsPosition(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  return evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
      deltaTs).deltaPij();
}

Vector3 evaluatePreintegratedMeasurementsVelocity(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  return evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
      deltaTs).deltaVij();
}

Rot3 evaluatePreintegratedMeasurementsRotation(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  return Rot3(
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
          deltaTs).deltaRij());
}

}

/* ************************************************************************* */
TEST( CombinedImuFactor, PreintegratedMeasurements ) {
  // Linearization point
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); ///< Current estimate of acceleration and angular rate biases

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;
  double tol = 1e-6;

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements expected1(bias, Matrix3::Zero(),
      Matrix3::Zero(), Matrix3::Zero());
  expected1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedImuFactor::CombinedPreintegratedMeasurements actual1(bias,
      Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),
      Matrix3::Zero(), Matrix::Zero(6, 6));

  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(
      assert_equal(Vector(expected1.deltaPij()), Vector(actual1.deltaPij()),
          tol));
  EXPECT(
      assert_equal(Vector(expected1.deltaVij()), Vector(actual1.deltaVij()),
          tol));
  EXPECT(
      assert_equal(Matrix(expected1.deltaRij()), Matrix(actual1.deltaRij()),
          tol));
  DOUBLES_EQUAL(expected1.deltaTij(), actual1.deltaTij(), tol);
}

/* ************************************************************************* */
TEST( CombinedImuFactor, ErrorWithBiases ) {
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)
  imuBias::ConstantBias bias2(Vector3(0.2, 0.2, 0), Vector3(1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)), Point3(5.0, 1.0, -50.0));
  Vector3 v1(0.5, 0.0, 0.0);
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(0.5, 0.0, 0.0);

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0.1, 0.1;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector()
      + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;
  double tol = 1e-6;

  Matrix I6x6(6, 6);
  I6x6 = Matrix::Identity(6, 6);

  ImuFactor::PreintegratedMeasurements pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
      Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity());

  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedImuFactor::CombinedPreintegratedMeasurements Combined_pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
      Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity(),
      Matrix3::Identity(), 2 * Matrix3::Identity(), I6x6);

  Combined_pre_int_data.integrateMeasurement(measuredAcc, measuredOmega,
      deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis);

  noiseModel::Gaussian::shared_ptr Combinedmodel =
      noiseModel::Gaussian::Covariance(Combined_pre_int_data.preintMeasCov());
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2),
      Combined_pre_int_data, gravity, omegaCoriolis);

  Vector errorExpected = factor.evaluateError(x1, v1, x2, v2, bias);

  Vector errorActual = Combinedfactor.evaluateError(x1, v1, x2, v2, bias,
      bias2);

  EXPECT(assert_equal(errorExpected, errorActual.head(9), tol));

  // Expected Jacobians
  Matrix H1e, H2e, H3e, H4e, H5e;
  (void) factor.evaluateError(x1, v1, x2, v2, bias, H1e, H2e, H3e, H4e, H5e);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a, H6a;
  (void) Combinedfactor.evaluateError(x1, v1, x2, v2, bias, bias2, H1a, H2a,
      H3a, H4a, H5a, H6a);

  EXPECT(assert_equal(H1e, H1a.topRows(9)));
  EXPECT(assert_equal(H2e, H2a.topRows(9)));
  EXPECT(assert_equal(H3e, H3a.topRows(9)));
  EXPECT(assert_equal(H4e, H4a.topRows(9)));
  EXPECT(assert_equal(H5e, H5a.topRows(9)));
}

/* ************************************************************************* */
TEST( CombinedImuFactor, FirstOrderPreIntegratedMeasurements ) {
  // Linearization point
  imuBias::ConstantBias bias; ///< Current estimate of acceleration and rotation rate biases

  Pose3 body_P_sensor(Rot3::Expmap(Vector3(0, 0.1, 0.1)), Point3(1, 0, 1));

  // Measurements
  list<Vector3> measuredAccs, measuredOmegas;
  list<double> deltaTs;
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  for (int i = 1; i < 100; i++) {
    measuredAccs.push_back(Vector3(0.05, 0.09, 0.01));
    measuredOmegas.push_back(
        Vector3(M_PI / 100.0, M_PI / 300.0, 2 * M_PI / 100.0));
    deltaTs.push_back(0.01);
  }

  // Actual preintegrated values
  CombinedImuFactor::CombinedPreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
          deltaTs);

  // Compute numerical derivatives
  Matrix expectedDelPdelBias = numericalDerivative11<Vector,
      imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsPosition, _1, measuredAccs,
          measuredOmegas, deltaTs), bias);
  Matrix expectedDelPdelBiasAcc = expectedDelPdelBias.leftCols(3);
  Matrix expectedDelPdelBiasOmega = expectedDelPdelBias.rightCols(3);

  Matrix expectedDelVdelBias = numericalDerivative11<Vector,
      imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsVelocity, _1, measuredAccs,
          measuredOmegas, deltaTs), bias);
  Matrix expectedDelVdelBiasAcc = expectedDelVdelBias.leftCols(3);
  Matrix expectedDelVdelBiasOmega = expectedDelVdelBias.rightCols(3);

  Matrix expectedDelRdelBias =
      numericalDerivative11<Rot3, imuBias::ConstantBias>(
          boost::bind(&evaluatePreintegratedMeasurementsRotation, _1,
              measuredAccs, measuredOmegas, deltaTs), bias);
  Matrix expectedDelRdelBiasAcc = expectedDelRdelBias.leftCols(3);
  Matrix expectedDelRdelBiasOmega = expectedDelRdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelPdelBiasAcc, preintegrated.delPdelBiasAcc()));
  EXPECT(
      assert_equal(expectedDelPdelBiasOmega, preintegrated.delPdelBiasOmega()));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, preintegrated.delVdelBiasAcc()));
  EXPECT(
      assert_equal(expectedDelVdelBiasOmega, preintegrated.delVdelBiasOmega()));
  EXPECT(assert_equal(expectedDelRdelBiasAcc, Matrix::Zero(3, 3)));
  EXPECT(
      assert_equal(expectedDelRdelBiasOmega, preintegrated.delRdelBiasOmega(),
          1e-3)); // 1e-3 needs to be added only when using quaternions for rotations
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictPositionAndVelocity) {
  imuBias::ConstantBias bias(Vector3(0, 0.1, 0), Vector3(0, 0.1, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << 0, 0.1, 0; //M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 1.1, -9.81;
  double deltaT = 0.001;

  Matrix I6x6(6, 6);
  I6x6 = Matrix::Identity(6, 6);

  CombinedImuFactor::CombinedPreintegratedMeasurements Combined_pre_int_data(
      bias, Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity(),
      Matrix3::Identity(), 2 * Matrix3::Identity(), I6x6, true);

  for (int i = 0; i < 1000; ++i)
    Combined_pre_int_data.integrateMeasurement(measuredAcc, measuredOmega,
        deltaT);

  // Create factor
  noiseModel::Gaussian::shared_ptr Combinedmodel =
      noiseModel::Gaussian::Covariance(Combined_pre_int_data.preintMeasCov());
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2),
      Combined_pre_int_data, gravity, omegaCoriolis);

  // Predict
  Pose3 x1;
  Vector3 v1(0, 0.0, 0.0);
  PoseVelocityBias poseVelocityBias = Combined_pre_int_data.predict(x1, v1,
      bias, gravity, omegaCoriolis);
  Pose3 expectedPose(Rot3(), Point3(0, 0.5, 0));
  Vector3 expectedVelocity;
  expectedVelocity << 0, 1, 0;
  EXPECT(assert_equal(expectedPose, poseVelocityBias.pose));
  EXPECT(
      assert_equal(Vector(expectedVelocity), Vector(poseVelocityBias.velocity)));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictRotation) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)
  Matrix I6x6(6, 6);
  CombinedImuFactor::CombinedPreintegratedMeasurements Combined_pre_int_data(
      bias, Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity(),
      Matrix3::Identity(), 2 * Matrix3::Identity(), I6x6, true);
  Vector3 measuredAcc;
  measuredAcc << 0, 0, -9.81;
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0;
  double deltaT = 0.001;
  double tol = 1e-4;
  for (int i = 0; i < 1000; ++i)
    Combined_pre_int_data.integrateMeasurement(measuredAcc, measuredOmega,
        deltaT);
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2),
      Combined_pre_int_data, gravity, omegaCoriolis);

  // Predict
  Pose3 x(Rot3().ypr(0, 0, 0), Point3(0, 0, 0)), x2;
  Vector3 v(0, 0, 0), v2;
  CombinedImuFactor::Predict(x, v, x2, v2, bias,
      Combinedfactor.preintegratedMeasurements(), gravity, omegaCoriolis);
  Pose3 expectedPose(Rot3().ypr(M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, x2, tol));
}

/* ************************************************************************* */
TEST( CombinedImuFactor, JacobianPreintegratedCovariancePropagation ) {
  // Linearization point
  imuBias::ConstantBias bias_old = imuBias::ConstantBias(); ///< Current estimate of acceleration and rotation rate biases
  Pose3 body_P_sensor = Pose3();

  // Measurements
  list<Vector3> measuredAccs, measuredOmegas;
  list<double> deltaTs;
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  for (int i = 1; i < 100; i++) {
    measuredAccs.push_back(Vector3(0.05, 0.09, 0.01));
    measuredOmegas.push_back(
        Vector3(M_PI / 100.0, M_PI / 300.0, 2 * M_PI / 100.0));
    deltaTs.push_back(0.01);
  }
  // Actual preintegrated values
  CombinedImuFactor::CombinedPreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(bias_old, measuredAccs, measuredOmegas,
          deltaTs);

  // so far we only created a nontrivial linearization point for the preintegrated measurements
  // Now we add a new measurement and ask for Jacobians
  const Vector3 newMeasuredAcc = Vector3(0.1, 0.0, 0.0);
  const Vector3 newMeasuredOmega = Vector3(M_PI / 100.0, 0.0, 0.0);
  const double newDeltaT = 0.01;
  const Rot3 deltaRij_old = preintegrated.deltaRij(); // before adding new measurement
  const Vector3 deltaVij_old = preintegrated.deltaVij(); // before adding new measurement
  const Vector3 deltaPij_old = preintegrated.deltaPij(); // before adding new measurement

  Matrix oldPreintCovariance = preintegrated.preintMeasCov();

  Matrix Factual, Gactual;
  preintegrated.integrateMeasurement(newMeasuredAcc, newMeasuredOmega,
      newDeltaT, body_P_sensor, Factual, Gactual);

  bool use2ndOrderIntegration = false;

  //////////////////////////////////////////////////////////////////////////////////////////////
  // COMPUTE NUMERICAL DERIVATIVES FOR F
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Compute expected F wrt positions (15,3)
  Matrix df_dpos = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsTest, _1, deltaVij_old,
          deltaRij_old, bias_old, newMeasuredAcc, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), deltaPij_old);
  // rotation part has to be done properly, on manifold
  df_dpos.block<3, 3>(6, 0) = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsRot, _1, deltaVij_old,
          deltaRij_old, bias_old, newMeasuredAcc, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), deltaPij_old);

  // Compute expected F wrt velocities (15,3)
  Matrix df_dvel = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsTest, deltaPij_old, _1,
          deltaRij_old, bias_old, newMeasuredAcc, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), deltaVij_old);
  // rotation part has to be done properly, on manifold
  df_dvel.block<3, 3>(6, 0) = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsRot, deltaPij_old, _1,
          deltaRij_old, bias_old, newMeasuredAcc, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), deltaVij_old);

  // Compute expected F wrt angles (15,3)
  Matrix df_dangle = numericalDerivative11<Vector, Rot3>(
      boost::bind(&updatePreintegratedMeasurementsTest, deltaPij_old,
          deltaVij_old, _1, bias_old, newMeasuredAcc, newMeasuredOmega,
          newDeltaT, use2ndOrderIntegration), deltaRij_old);
  // rotation part has to be done properly, on manifold
  df_dangle.block<3, 3>(6, 0) = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&updatePreintegratedMeasurementsRot, deltaPij_old,
          deltaVij_old, _1, bias_old, newMeasuredAcc, newMeasuredOmega,
          newDeltaT, use2ndOrderIntegration), deltaRij_old);

  // Compute expected F wrt biases (15,6)
  Matrix df_dbias = numericalDerivative11<Vector, imuBias::ConstantBias>(
      boost::bind(&updatePreintegratedMeasurementsTest, deltaPij_old,
          deltaVij_old, deltaRij_old, _1, newMeasuredAcc, newMeasuredOmega,
          newDeltaT, use2ndOrderIntegration), bias_old);
  // rotation part has to be done properly, on manifold
  df_dbias.block<3, 6>(6, 0) =
      numericalDerivative11<Rot3, imuBias::ConstantBias>(
          boost::bind(&updatePreintegratedMeasurementsRot, deltaPij_old,
              deltaVij_old, deltaRij_old, _1, newMeasuredAcc, newMeasuredOmega,
              newDeltaT, use2ndOrderIntegration), bias_old);

  Matrix Fexpected(15, 15);
  Fexpected << df_dpos, df_dvel, df_dangle, df_dbias;
  EXPECT(assert_equal(Fexpected, Factual));

  //////////////////////////////////////////////////////////////////////////////////////////////
  // COMPUTE NUMERICAL DERIVATIVES FOR G
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Compute expected G wrt integration noise
  Matrix df_dintNoise(15, 3);
  df_dintNoise << I_3x3 * newDeltaT, Z_3x3, Z_3x3, Z_3x3, Z_3x3;

  // Compute expected G wrt acc noise (15,3)
  Matrix df_daccNoise = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsTest, deltaPij_old,
          deltaVij_old, deltaRij_old, bias_old, _1, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), newMeasuredAcc);
  // rotation part has to be done properly, on manifold
  df_daccNoise.block<3, 3>(6, 0) = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsRot, deltaPij_old,
          deltaVij_old, deltaRij_old, bias_old, _1, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), newMeasuredAcc);

  // Compute expected G wrt gyro noise (15,3)
  Matrix df_domegaNoise = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsTest, deltaPij_old,
          deltaVij_old, deltaRij_old, bias_old, newMeasuredAcc, _1, newDeltaT,
          use2ndOrderIntegration), newMeasuredOmega);
  // rotation part has to be done properly, on manifold
  df_domegaNoise.block<3, 3>(6, 0) = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&updatePreintegratedMeasurementsRot, deltaPij_old,
          deltaVij_old, deltaRij_old, bias_old, newMeasuredAcc, _1, newDeltaT,
          use2ndOrderIntegration), newMeasuredOmega);

  // Compute expected G wrt bias random walk noise (15,6)
  Matrix df_rwBias(15, 6); // random walk on the bias does not appear in the first 9 entries
  df_rwBias.setZero();
  df_rwBias.block<6, 6>(9, 0) = eye(6);

  // Compute expected G wrt gyro noise (15,6)
  Matrix df_dinitBias = numericalDerivative11<Vector, imuBias::ConstantBias>(
      boost::bind(&updatePreintegratedMeasurementsTest, deltaPij_old,
          deltaVij_old, deltaRij_old, _1, newMeasuredAcc, newMeasuredOmega,
          newDeltaT, use2ndOrderIntegration), bias_old);
  // rotation part has to be done properly, on manifold
  df_dinitBias.block<3, 6>(6, 0) = numericalDerivative11<Rot3,
      imuBias::ConstantBias>(
      boost::bind(&updatePreintegratedMeasurementsRot, deltaPij_old,
          deltaVij_old, deltaRij_old, _1, newMeasuredAcc, newMeasuredOmega,
          newDeltaT, use2ndOrderIntegration), bias_old);
  df_dinitBias.block<6, 6>(9, 0) = Matrix::Zero(6, 6); // only has to influence first 9 rows

  Matrix Gexpected(15, 21);
  Gexpected << df_dintNoise, df_daccNoise, df_domegaNoise, df_rwBias, df_dinitBias;

  EXPECT(assert_equal(Gexpected, Gactual));

  // Check covariance propagation
  Matrix newPreintCovarianceExpected = Factual * oldPreintCovariance
      * Factual.transpose() + (1 / newDeltaT) * Gactual * Gactual.transpose();

  Matrix newPreintCovarianceActual = preintegrated.preintMeasCov();
  EXPECT(assert_equal(newPreintCovarianceExpected, newPreintCovarianceActual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
