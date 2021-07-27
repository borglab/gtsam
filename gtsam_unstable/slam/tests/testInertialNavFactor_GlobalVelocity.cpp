/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInertialNavFactor_GlobalVelocity.cpp
 * @brief   Unit test for the InertialNavFactor_GlobalVelocity
 * @author  Vadim Indelman, Stephen Williams
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam_unstable/slam/InertialNavFactor_GlobalVelocity.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

Rot3 world_R_ECEF(0.31686, 0.51505, 0.79645, 0.85173, -0.52399, 0, 0.41733,
    0.67835, -0.60471);

static const Vector3 world_g(0.0, 0.0, 9.81);
static const Vector3 world_rho(0.0, -1.5724e-05, 0.0); // NED system
static const Vector3 ECEF_omega_earth(0.0, 0.0, 7.292115e-5);
static const Vector3 world_omega_earth = world_R_ECEF.matrix()
    * ECEF_omega_earth;

/* ************************************************************************* */
Pose3 predictionErrorPose(const Pose3& p1, const Vector3& v1,
    const imuBias::ConstantBias& b1, const Pose3& p2, const Vector3& v2,
    const InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias>& factor) {
  return Pose3::Expmap(factor.evaluateError(p1, v1, b1, p2, v2).head(6));
}

Vector predictionErrorVel(const Pose3& p1, const Vector3& v1,
    const imuBias::ConstantBias& b1, const Pose3& p2, const Vector3& v2,
    const InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias>& factor) {
  return factor.evaluateError(p1, v1, b1, p2, v2).tail(3);
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, Constructor) {
  Key Pose1(11);
  Key Pose2(12);
  Key Vel1(21);
  Key Vel2(22);
  Key Bias1(31);

  Vector3 measurement_acc(0.1, 0.2, 0.4);
  Vector3 measurement_gyro(-0.2, 0.5, 0.03);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro,
      measurement_dt, world_g, world_rho, world_omega_earth, model);
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, Equals) {
  Key Pose1(11);
  Key Pose2(12);
  Key Vel1(21);
  Key Vel2(22);
  Key Bias1(31);

  Vector3 measurement_acc(0.1, 0.2, 0.4);
  Vector3 measurement_gyro(-0.2, 0.5, 0.03);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro,
      measurement_dt, world_g, world_rho, world_omega_earth, model);
  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> g(
      Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro,
      measurement_dt, world_g, world_rho, world_omega_earth, model);
  CHECK(assert_equal(f, g, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, Predict) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // First test: zero angular motion, some acceleration
  Vector measurement_acc(Vector3(0.1, 0.2, 0.3 - 9.81));
  Vector measurement_gyro(Vector3(0.0, 0.0, 0.0));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  Vector3 Vel1(Vector3(0.50, -0.50, 0.40));
  imuBias::ConstantBias Bias1;
  Pose3 expectedPose2(Rot3(), Point3(2.05, 0.95, 3.04));
  Vector3 expectedVel2(Vector3(0.51, -0.48, 0.43));
  Pose3 actualPose2;
  Vector3 actualVel2;
  f.predict(Pose1, Vel1, Bias1, actualPose2, actualVel2);

  CHECK(assert_equal(expectedPose2, actualPose2, 1e-5));
  CHECK(assert_equal((Vector)expectedVel2, actualVel2, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ErrorPosVel) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // First test: zero angular motion, some acceleration
  Vector measurement_acc(Vector3(0.1, 0.2, 0.3 - 9.81));
  Vector measurement_gyro(Vector3(0.0, 0.0, 0.0));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  Pose3 Pose2(Rot3(), Point3(2.05, 0.95, 3.04));
  Vector3 Vel1(Vector3(0.50, -0.50, 0.40));
  Vector3 Vel2(Vector3(0.51, -0.48, 0.43));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(Z_9x1);

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ErrorRot) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // Second test: zero angular motion, some acceleration
  Vector measurement_acc(Vector3(0.0, 0.0, 0.0 - 9.81));
  Vector measurement_gyro(Vector3(0.1, 0.2, 0.3));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model);

  Pose3 Pose1(Rot3(), Point3(2.0, 1.0, 3.0));
  Pose3 Pose2(Rot3::Expmap(measurement_gyro * measurement_dt),
      Point3(2.0, 1.0, 3.0));
  Vector3 Vel1(Vector3(0.0, 0.0, 0.0));
  Vector3 Vel2(Vector3(0.0, 0.0, 0.0));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(Z_9x1);

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ErrorRotPosVel) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // Second test: zero angular motion, some acceleration - generated in matlab
  Vector measurement_acc(
      Vector3(6.501390843381716, -6.763926150509185, -2.300389940090343));
  Vector measurement_gyro(Vector3(0.1, 0.2, 0.3));

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model);

  Rot3 R1(0.487316618, 0.125253866, 0.86419557, 0.580273724, 0.693095498,
      -0.427669306, -0.652537293, 0.709880342, 0.265075427);
  Point3 t1(2.0, 1.0, 3.0);
  Pose3 Pose1(R1, t1);
  Vector3 Vel1(Vector3(0.5, -0.5, 0.4));
  Rot3 R2(0.473618898, 0.119523052, 0.872582019, 0.609241153, 0.67099888,
      -0.422594037, -0.636011287, 0.731761397, 0.244979388);
  Point3 t2 = t1 + Point3(Vel1 * measurement_dt);
  Pose3 Pose2(R2, t2);
  Vector dv = measurement_dt * (R1.matrix() * measurement_acc + world_g);
  Vector3 Vel2 = Vel1 + dv;
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(Z_9x1);

  // TODO: Expected values need to be updated for global velocity version
  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

///* VADIM - START ************************************************************************* */
//Vector3 predictionRq(const Vector3 angles, const Vector3 q) {
//  return (Rot3().RzRyRx(angles) * q);
//}
//
//TEST (InertialNavFactor_GlobalVelocity, Rotation_Deriv ) {
//  Vector3 angles(Vector3(3.001, -1.0004, 2.0005));
//  Rot3 R1(Rot3().RzRyRx(angles));
//  Vector3 q(Vector3(5.8, -2.2, 4.105));
//  Rot3 qx(0.0, -q[2], q[1],
//      q[2], 0.0, -q[0],
//      -q[1], q[0],0.0);
//  Matrix J_hyp( -(R1*qx).matrix() );
//
//  Matrix J_expected;
//
//  Vector3 v(predictionRq(angles, q));
//
//  J_expected = numericalDerivative11<Vector3, Vector3>(std::bind(&predictionRq, std::placeholders::_1, q), angles);
//
//  cout<<"J_hyp"<<J_hyp<<endl;
//  cout<<"J_expected"<<J_expected<<endl;
//
//  CHECK( assert_equal(J_expected, J_hyp, 1e-6));
//}
///* VADIM - END ************************************************************************* */

/* ************************************************************************* */TEST (InertialNavFactor_GlobalVelocity, Jacobian ) {

  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.01);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Vector measurement_acc(
      Vector3(6.501390843381716, -6.763926150509185, -2.300389940090343));
  Vector measurement_gyro((Vector(3) << 3.14, 3.14 / 2, -3.14).finished());

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> factor(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model);

  Rot3 R1(0.487316618, 0.125253866, 0.86419557, 0.580273724, 0.693095498,
      -0.427669306, -0.652537293, 0.709880342, 0.265075427);
  Point3 t1(2.0, 1.0, 3.0);
  Pose3 Pose1(R1, t1);
  Vector3 Vel1(Vector3(0.5, -0.5, 0.4));
  Rot3 R2(0.473618898, 0.119523052, 0.872582019, 0.609241153, 0.67099888,
      -0.422594037, -0.636011287, 0.731761397, 0.244979388);
  Point3 t2(2.052670960415706, 0.977252139079380, 2.942482135362800);
  Pose3 Pose2(R2, t2);
  Vector3 Vel2(
      Vector3(0.510000000000000, -0.480000000000000, 0.430000000000000));
  imuBias::ConstantBias Bias1;

  Matrix H1_actual, H2_actual, H3_actual, H4_actual, H5_actual;

  Vector ActualErr(
      factor.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2, H1_actual,
          H2_actual, H3_actual, H4_actual, H5_actual));

  // Checking for Pose part in the jacobians
  // ******
  Matrix H1_actualPose(H1_actual.block(0, 0, 6, H1_actual.cols()));
  Matrix H2_actualPose(H2_actual.block(0, 0, 6, H2_actual.cols()));
  Matrix H3_actualPose(H3_actual.block(0, 0, 6, H3_actual.cols()));
  Matrix H4_actualPose(H4_actual.block(0, 0, 6, H4_actual.cols()));
  Matrix H5_actualPose(H5_actual.block(0, 0, 6, H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  Matrix H1_expectedPose, H2_expectedPose, H3_expectedPose, H4_expectedPose,
      H5_expectedPose;
  H1_expectedPose = numericalDerivative11<Pose3, Pose3>(
      std::bind(&predictionErrorPose, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2, factor),
      Pose1);
  H2_expectedPose = numericalDerivative11<Pose3, Vector3>(
      std::bind(&predictionErrorPose, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2, factor),
      Vel1);
  H3_expectedPose = numericalDerivative11<Pose3, imuBias::ConstantBias>(
      std::bind(&predictionErrorPose, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2, factor),
      Bias1);
  H4_expectedPose = numericalDerivative11<Pose3, Pose3>(
      std::bind(&predictionErrorPose, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2, factor),
      Pose2);
  H5_expectedPose = numericalDerivative11<Pose3, Vector3>(
      std::bind(&predictionErrorPose, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1, factor),
      Vel2);

  // Verify they are equal for this choice of state
  CHECK( assert_equal(H1_expectedPose, H1_actualPose, 1e-5));
  CHECK( assert_equal(H2_expectedPose, H2_actualPose, 1e-5));
  CHECK( assert_equal(H3_expectedPose, H3_actualPose, 2e-3));
  CHECK( assert_equal(H4_expectedPose, H4_actualPose, 1e-5));
  CHECK( assert_equal(H5_expectedPose, H5_actualPose, 1e-5));

  // Checking for Vel part in the jacobians
  // ******
  Matrix H1_actualVel(H1_actual.block(6, 0, 3, H1_actual.cols()));
  Matrix H2_actualVel(H2_actual.block(6, 0, 3, H2_actual.cols()));
  Matrix H3_actualVel(H3_actual.block(6, 0, 3, H3_actual.cols()));
  Matrix H4_actualVel(H4_actual.block(6, 0, 3, H4_actual.cols()));
  Matrix H5_actualVel(H5_actual.block(6, 0, 3, H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  Matrix H1_expectedVel, H2_expectedVel, H3_expectedVel, H4_expectedVel,
      H5_expectedVel;
  H1_expectedVel = numericalDerivative11<Vector, Pose3>(
      std::bind(&predictionErrorVel, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2, factor),
      Pose1);
  H2_expectedVel = numericalDerivative11<Vector, Vector3>(
      std::bind(&predictionErrorVel, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2, factor),
      Vel1);
  H3_expectedVel = numericalDerivative11<Vector, imuBias::ConstantBias>(
      std::bind(&predictionErrorVel, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2, factor),
      Bias1);
  H4_expectedVel = numericalDerivative11<Vector, Pose3>(
      std::bind(&predictionErrorVel, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2, factor),
      Pose2);
  H5_expectedVel = numericalDerivative11<Vector, Vector3>(
      std::bind(&predictionErrorVel, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1, factor),
      Vel2);

  // Verify they are equal for this choice of state
  CHECK( assert_equal(H1_expectedVel, H1_actualVel, 1e-5));
  CHECK( assert_equal(H2_expectedVel, H2_actualVel, 1e-5));
  CHECK( assert_equal(H3_expectedVel, H3_actualVel, 1e-5));
  CHECK( assert_equal(H4_expectedVel, H4_actualVel, 1e-5));
  CHECK( assert_equal(H5_expectedVel, H5_actualVel, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ConstructorWithTransform) {
  Key Pose1(11);
  Key Pose2(12);
  Key Vel1(21);
  Key Vel2(22);
  Key Bias1(31);

  Vector measurement_acc(Vector3(0.1, 0.2, 0.4));
  Vector measurement_gyro(Vector3(-0.2, 0.5, 0.03));

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(0.0, 0.0, 0.0)); // IMU is in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro,
      measurement_dt, world_g, world_rho, world_omega_earth, model,
      body_P_sensor);
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, EqualsWithTransform) {
  Key Pose1(11);
  Key Pose2(12);
  Key Vel1(21);
  Key Vel2(22);
  Key Bias1(31);

  Vector measurement_acc(Vector3(0.1, 0.2, 0.4));
  Vector measurement_gyro(Vector3(-0.2, 0.5, 0.03));

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(0.0, 0.0, 0.0)); // IMU is in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro,
      measurement_dt, world_g, world_rho, world_omega_earth, model,
      body_P_sensor);
  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> g(
      Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro,
      measurement_dt, world_g, world_rho, world_omega_earth, model,
      body_P_sensor);
  CHECK(assert_equal(f, g, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, PredictWithTransform) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0)); // IMU is in ENU orientation

  // First test: zero angular motion, some acceleration
  Vector measurement_gyro(Vector3(0.0, 0.0, 0.0)); // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = Vector3(0.2, 0.1, -0.3 + 9.81)
      + omega__cross * omega__cross
          * body_P_sensor.rotation().inverse().matrix()
          * body_P_sensor.translation(); // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model, body_P_sensor);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  Vector3 Vel1(Vector3(0.50, -0.50, 0.40));
  imuBias::ConstantBias Bias1;
  Pose3 expectedPose2(Rot3(), Point3(2.05, 0.95, 3.04));
  Vector3 expectedVel2(Vector3(0.51, -0.48, 0.43));
  Pose3 actualPose2;
  Vector3 actualVel2;
  f.predict(Pose1, Vel1, Bias1, actualPose2, actualVel2);

  CHECK(assert_equal(expectedPose2, actualPose2, 1e-5));
  CHECK(assert_equal((Vector)expectedVel2, actualVel2, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ErrorPosVelWithTransform) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0)); // IMU is in ENU orientation

  // First test: zero angular motion, some acceleration
  Vector measurement_gyro(Vector3(0.0, 0.0, 0.0)); // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = Vector3(0.2, 0.1, -0.3 + 9.81)
      + omega__cross * omega__cross
          * body_P_sensor.rotation().inverse().matrix()
          * body_P_sensor.translation(); // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model, body_P_sensor);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  Pose3 Pose2(Rot3(), Point3(2.05, 0.95, 3.04));
  Vector3 Vel1(Vector3(0.50, -0.50, 0.40));
  Vector3 Vel2(Vector3(0.51, -0.48, 0.43));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(Z_9x1);

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ErrorRotWithTransform) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0)); // IMU is in ENU orientation

  // Second test: zero angular motion, some acceleration
  Vector measurement_gyro(Vector3(0.2, 0.1, -0.3)); // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = Vector3(0.0, 0.0, 0.0 + 9.81)
      + omega__cross * omega__cross
          * body_P_sensor.rotation().inverse().matrix()
          * body_P_sensor.translation(); // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model, body_P_sensor);

  Pose3 Pose1(Rot3(), Point3(2.0, 1.0, 3.0));
  Pose3 Pose2(
      Rot3::Expmap(
          body_P_sensor.rotation().matrix() * measurement_gyro
              * measurement_dt), Point3(2.0, 1.0, 3.0));
  Vector3 Vel1(Vector3(0.0, 0.0, 0.0));
  Vector3 Vel2(Vector3(0.0, 0.0, 0.0));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(Z_9x1);

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */TEST( InertialNavFactor_GlobalVelocity, ErrorRotPosVelWithTransform) {
  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.1);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0)); // IMU is in ENU orientation

  // Second test: zero angular motion, some acceleration - generated in matlab
  Vector measurement_gyro(Vector3(0.2, 0.1, -0.3)); // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc =
      Vector3(-6.763926150509185, 6.501390843381716, +2.300389940090343)
          + omega__cross * omega__cross
              * body_P_sensor.rotation().inverse().matrix()
              * body_P_sensor.translation(); // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> f(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model, body_P_sensor);

  Rot3 R1(0.487316618, 0.125253866, 0.86419557, 0.580273724, 0.693095498,
      -0.427669306, -0.652537293, 0.709880342, 0.265075427);
  Point3 t1(2.0, 1.0, 3.0);
  Pose3 Pose1(R1, t1);
  Vector3 Vel1(Vector3(0.5, -0.5, 0.4));
  Rot3 R2(0.473618898, 0.119523052, 0.872582019, 0.609241153, 0.67099888,
      -0.422594037, -0.636011287, 0.731761397, 0.244979388);
  Point3 t2 = t1+ Point3(Vel1 * measurement_dt);
  Pose3 Pose2(R2, t2);
  Vector dv =
      measurement_dt
          * (R1.matrix() * body_P_sensor.rotation().matrix()
              * Vector3(-6.763926150509185, 6.501390843381716, +2.300389940090343)
              + world_g);
  Vector3 Vel2 = Vel1 + dv;
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(Z_9x1);

  // TODO: Expected values need to be updated for global velocity version
  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */TEST (InertialNavFactor_GlobalVelocity, JacobianWithTransform ) {

  Key PoseKey1(11);
  Key PoseKey2(12);
  Key VelKey1(21);
  Key VelKey2(22);
  Key BiasKey1(31);

  double measurement_dt(0.01);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0)); // IMU is in ENU orientation

  Vector measurement_gyro((Vector(3) << 3.14 / 2, 3.14, +3.14).finished()); // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc =
      Vector3(-6.763926150509185, 6.501390843381716, +2.300389940090343)
          + omega__cross * omega__cross
              * body_P_sensor.rotation().inverse().matrix()
              * body_P_sensor.translation(); // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, Vector3, imuBias::ConstantBias> factor(
      PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc,
      measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth,
      model, body_P_sensor);

  Rot3 R1(0.487316618, 0.125253866, 0.86419557, 0.580273724, 0.693095498,
      -0.427669306, -0.652537293, 0.709880342, 0.265075427);
  Point3 t1(2.0, 1.0, 3.0);
  Pose3 Pose1(R1, t1);
  Vector3 Vel1(0.5, -0.5, 0.4);
  Rot3 R2(0.473618898, 0.119523052, 0.872582019, 0.609241153, 0.67099888,
      -0.422594037, -0.636011287, 0.731761397, 0.244979388);
  Point3 t2(2.052670960415706, 0.977252139079380, 2.942482135362800);
  Pose3 Pose2(R2, t2);
  Vector3 Vel2(0.510000000000000, -0.480000000000000, 0.430000000000000);
  imuBias::ConstantBias Bias1;

  Matrix H1_actual, H2_actual, H3_actual, H4_actual, H5_actual;

  Vector ActualErr(
      factor.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2, H1_actual,
          H2_actual, H3_actual, H4_actual, H5_actual));

  // Checking for Pose part in the jacobians
  // ******
  Matrix H1_actualPose(H1_actual.block(0, 0, 6, H1_actual.cols()));
  Matrix H2_actualPose(H2_actual.block(0, 0, 6, H2_actual.cols()));
  Matrix H3_actualPose(H3_actual.block(0, 0, 6, H3_actual.cols()));
  Matrix H4_actualPose(H4_actual.block(0, 0, 6, H4_actual.cols()));
  Matrix H5_actualPose(H5_actual.block(0, 0, 6, H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  Matrix H1_expectedPose, H2_expectedPose, H3_expectedPose, H4_expectedPose,
      H5_expectedPose;
  H1_expectedPose = numericalDerivative11<Pose3, Pose3>(
      std::bind(&predictionErrorPose, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2, factor),
      Pose1);
  H2_expectedPose = numericalDerivative11<Pose3, Vector3>(
      std::bind(&predictionErrorPose, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2, factor),
      Vel1);
  H3_expectedPose = numericalDerivative11<Pose3, imuBias::ConstantBias>(
      std::bind(&predictionErrorPose, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2, factor),
      Bias1);
  H4_expectedPose = numericalDerivative11<Pose3, Pose3>(
      std::bind(&predictionErrorPose, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2, factor),
      Pose2);
  H5_expectedPose = numericalDerivative11<Pose3, Vector3>(
      std::bind(&predictionErrorPose, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1, factor),
      Vel2);

  // Verify they are equal for this choice of state
  CHECK( assert_equal(H1_expectedPose, H1_actualPose, 1e-5));
  CHECK( assert_equal(H2_expectedPose, H2_actualPose, 1e-5));
  CHECK( assert_equal(H3_expectedPose, H3_actualPose, 2e-3));
  CHECK( assert_equal(H4_expectedPose, H4_actualPose, 1e-5));
  CHECK( assert_equal(H5_expectedPose, H5_actualPose, 1e-5));

  // Checking for Vel part in the jacobians
  // ******
  Matrix H1_actualVel(H1_actual.block(6, 0, 3, H1_actual.cols()));
  Matrix H2_actualVel(H2_actual.block(6, 0, 3, H2_actual.cols()));
  Matrix H3_actualVel(H3_actual.block(6, 0, 3, H3_actual.cols()));
  Matrix H4_actualVel(H4_actual.block(6, 0, 3, H4_actual.cols()));
  Matrix H5_actualVel(H5_actual.block(6, 0, 3, H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  Matrix H1_expectedVel, H2_expectedVel, H3_expectedVel, H4_expectedVel,
      H5_expectedVel;
  H1_expectedVel = numericalDerivative11<Vector, Pose3>(
      std::bind(&predictionErrorVel, std::placeholders::_1, Vel1, Bias1, Pose2, Vel2, factor),
      Pose1);
  H2_expectedVel = numericalDerivative11<Vector, Vector3>(
      std::bind(&predictionErrorVel, Pose1, std::placeholders::_1, Bias1, Pose2, Vel2, factor),
      Vel1);
  H3_expectedVel = numericalDerivative11<Vector, imuBias::ConstantBias>(
      std::bind(&predictionErrorVel, Pose1, Vel1, std::placeholders::_1, Pose2, Vel2, factor),
      Bias1);
  H4_expectedVel = numericalDerivative11<Vector, Pose3>(
      std::bind(&predictionErrorVel, Pose1, Vel1, Bias1, std::placeholders::_1, Vel2, factor),
      Pose2);
  H5_expectedVel = numericalDerivative11<Vector, Vector3>(
      std::bind(&predictionErrorVel, Pose1, Vel1, Bias1, Pose2, std::placeholders::_1, factor),
      Vel2);

  // Verify they are equal for this choice of state
  CHECK( assert_equal(H1_expectedVel, H1_actualVel, 1e-5));
  CHECK( assert_equal(H2_expectedVel, H2_actualVel, 1e-5));
  CHECK( assert_equal(H3_expectedVel, H3_actualVel, 1e-5));
  CHECK( assert_equal(H4_expectedVel, H4_actualVel, 1e-5));
  CHECK( assert_equal(H5_expectedVel, H5_actualVel, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
