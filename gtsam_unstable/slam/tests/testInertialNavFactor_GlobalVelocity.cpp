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
#include <gtsam/base/LieVector.h>
#include <gtsam/base/TestableAssertions.h>

using namespace std;
using namespace gtsam;

gtsam::Rot3 world_R_ECEF(
    0.31686,      0.51505,      0.79645,
    0.85173,     -0.52399,            0,
    0.41733,      0.67835,     -0.60471);

gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

/* ************************************************************************* */
gtsam::Pose3 predictionErrorPose(const Pose3& p1, const LieVector& v1, const imuBias::ConstantBias& b1, const Pose3& p2, const LieVector& v2, const InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias>& factor) {
  return Pose3::Expmap(factor.evaluateError(p1, v1, b1, p2, v2).head(6));
}

gtsam::LieVector predictionErrorVel(const Pose3& p1, const LieVector& v1, const imuBias::ConstantBias& b1, const Pose3& p2, const LieVector& v2, const InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias>& factor) {
  return LieVector::Expmap(factor.evaluateError(p1, v1, b1, p2, v2).tail(3));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, Constructor)
{
  gtsam::Key Pose1(11);
  gtsam::Key Pose2(12);
  gtsam::Key Vel1(21);
  gtsam::Key Vel2(22);
  gtsam::Key Bias1(31);

  Vector measurement_acc((Vector(3) << 0.1,0.2,0.4));
  Vector measurement_gyro((Vector(3) << -0.2, 0.5, 0.03));

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, Equals)
{
  gtsam::Key Pose1(11);
  gtsam::Key Pose2(12);
  gtsam::Key Vel1(21);
  gtsam::Key Vel2(22);
  gtsam::Key Bias1(31);

  Vector measurement_acc((Vector(3) << 0.1,0.2,0.4));
  Vector measurement_gyro((Vector(3) << -0.2, 0.5, 0.03));

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> g(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);
  CHECK(assert_equal(f, g, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, Predict)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));


  // First test: zero angular motion, some acceleration
  Vector measurement_acc((Vector(3) <<0.1,0.2,0.3-9.81));
  Vector measurement_gyro((Vector(3) << 0.0, 0.0, 0.0));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  LieVector Vel1((Vector(3) << 0.50, -0.50, 0.40));
  imuBias::ConstantBias Bias1;
  Pose3 expectedPose2(Rot3(), Point3(2.05, 0.95, 3.04));
  LieVector expectedVel2((Vector(3) << 0.51, -0.48, 0.43));
  Pose3 actualPose2;
  LieVector actualVel2;
  f.predict(Pose1, Vel1, Bias1, actualPose2, actualVel2);

  CHECK(assert_equal(expectedPose2, actualPose2, 1e-5));
  CHECK(assert_equal(expectedVel2, actualVel2, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ErrorPosVel)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));


  // First test: zero angular motion, some acceleration
  Vector measurement_acc((Vector(3) <<0.1,0.2,0.3-9.81));
  Vector measurement_gyro((Vector(3) << 0.0, 0.0, 0.0));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  Pose3 Pose2(Rot3(), Point3(2.05, 0.95, 3.04));
  LieVector Vel1((Vector(3) << 0.50, -0.50, 0.40));
  LieVector Vel2((Vector(3) << 0.51, -0.48, 0.43));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(zero(9));

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ErrorRot)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // Second test: zero angular motion, some acceleration
  Vector measurement_acc((Vector(3) <<0.0,0.0,0.0-9.81));
  Vector measurement_gyro((Vector(3) << 0.1, 0.2, 0.3));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);

  Pose3 Pose1(Rot3(), Point3(2.0,1.0,3.0));
  Pose3 Pose2(Rot3::Expmap(measurement_gyro*measurement_dt), Point3(2.0,1.0,3.0));
  LieVector Vel1((Vector(3) << 0.0, 0.0, 0.0));
  LieVector Vel2((Vector(3) << 0.0, 0.0, 0.0));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(zero(9));

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ErrorRotPosVel)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  // Second test: zero angular motion, some acceleration - generated in matlab
  Vector measurement_acc((Vector(3) << 6.501390843381716,  -6.763926150509185,  -2.300389940090343));
  Vector measurement_gyro((Vector(3) << 0.1, 0.2, 0.3));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);

  Rot3 R1(0.487316618,   0.125253866,    0.86419557,
       0.580273724,   0.693095498,  -0.427669306,
      -0.652537293,   0.709880342,   0.265075427);
  Point3 t1(2.0,1.0,3.0);
  Pose3 Pose1(R1, t1);
  LieVector Vel1((Vector(3) << 0.5, -0.5, 0.4));
  Rot3 R2(0.473618898,   0.119523052,   0.872582019,
       0.609241153,    0.67099888,  -0.422594037,
      -0.636011287,   0.731761397,   0.244979388);
  Point3 t2 = t1.compose( Point3(Vel1*measurement_dt) );
  Pose3 Pose2(R2, t2);
  Vector dv = measurement_dt * (R1.matrix() * measurement_acc + world_g);
  LieVector Vel2 = Vel1.compose( dv );
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(zero(9));

  // TODO: Expected values need to be updated for global velocity version
  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}


///* VADIM - START ************************************************************************* */
//LieVector predictionRq(const LieVector angles, const LieVector q) {
//  return (Rot3().RzRyRx(angles) * q).vector();
//}
//
//TEST (InertialNavFactor_GlobalVelocity, Rotation_Deriv ) {
//  LieVector angles((Vector(3) << 3.001, -1.0004, 2.0005));
//  Rot3 R1(Rot3().RzRyRx(angles));
//  LieVector q((Vector(3) << 5.8, -2.2, 4.105));
//  Rot3 qx(0.0, -q[2], q[1],
//      q[2], 0.0, -q[0],
//      -q[1], q[0],0.0);
//  Matrix J_hyp( -(R1*qx).matrix() );
//
//  gtsam::Matrix J_expected;
//
//  LieVector v(predictionRq(angles, q));
//
//  J_expected = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionRq, _1, q), angles);
//
//  cout<<"J_hyp"<<J_hyp<<endl;
//  cout<<"J_expected"<<J_expected<<endl;
//
//  CHECK( gtsam::assert_equal(J_expected, J_hyp, 1e-6));
//}
///* VADIM - END ************************************************************************* */

/* ************************************************************************* */
TEST (InertialNavFactor_GlobalVelocity, Jacobian ) {

  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.01);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Vector measurement_acc((Vector(3) << 6.501390843381716,  -6.763926150509185,  -2.300389940090343));
  Vector measurement_gyro((Vector(3) << 3.14, 3.14/2, -3.14));

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> factor(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model);

  Rot3 R1(0.487316618,   0.125253866,    0.86419557,
       0.580273724,   0.693095498,  -0.427669306,
      -0.652537293,   0.709880342,   0.265075427);
  Point3 t1(2.0,1.0,3.0);
  Pose3 Pose1(R1, t1);
  LieVector Vel1((Vector(3) << 0.5, -0.5, 0.4));
  Rot3 R2(0.473618898,   0.119523052,   0.872582019,
       0.609241153,    0.67099888,  -0.422594037,
      -0.636011287,   0.731761397,   0.244979388);
  Point3 t2(2.052670960415706,   0.977252139079380,   2.942482135362800);
  Pose3 Pose2(R2, t2);
  LieVector Vel2((Vector(3) << 0.510000000000000,  -0.480000000000000,   0.430000000000000));
  imuBias::ConstantBias Bias1;

  Matrix H1_actual, H2_actual, H3_actual, H4_actual, H5_actual;

  Vector ActualErr(factor.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2, H1_actual, H2_actual, H3_actual, H4_actual, H5_actual));

  // Checking for Pose part in the jacobians
  // ******
  Matrix H1_actualPose(H1_actual.block(0,0,6,H1_actual.cols()));
  Matrix H2_actualPose(H2_actual.block(0,0,6,H2_actual.cols()));
  Matrix H3_actualPose(H3_actual.block(0,0,6,H3_actual.cols()));
  Matrix H4_actualPose(H4_actual.block(0,0,6,H4_actual.cols()));
  Matrix H5_actualPose(H5_actual.block(0,0,6,H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  gtsam::Matrix H1_expectedPose, H2_expectedPose, H3_expectedPose, H4_expectedPose, H5_expectedPose;
  H1_expectedPose = gtsam::numericalDerivative11<Pose3, Pose3>(boost::bind(&predictionErrorPose, _1, Vel1, Bias1, Pose2, Vel2, factor), Pose1);
  H2_expectedPose = gtsam::numericalDerivative11<Pose3, LieVector>(boost::bind(&predictionErrorPose, Pose1, _1, Bias1, Pose2, Vel2, factor), Vel1);
  H3_expectedPose = gtsam::numericalDerivative11<Pose3, imuBias::ConstantBias>(boost::bind(&predictionErrorPose, Pose1, Vel1, _1, Pose2, Vel2, factor), Bias1);
  H4_expectedPose = gtsam::numericalDerivative11<Pose3, Pose3>(boost::bind(&predictionErrorPose, Pose1, Vel1, Bias1, _1, Vel2, factor), Pose2);
  H5_expectedPose = gtsam::numericalDerivative11<Pose3, LieVector>(boost::bind(&predictionErrorPose, Pose1, Vel1, Bias1, Pose2, _1, factor), Vel2);

  // Verify they are equal for this choice of state
  CHECK( gtsam::assert_equal(H1_expectedPose, H1_actualPose, 1e-5));
  CHECK( gtsam::assert_equal(H2_expectedPose, H2_actualPose, 1e-5));
  CHECK( gtsam::assert_equal(H3_expectedPose, H3_actualPose, 2e-3));
  CHECK( gtsam::assert_equal(H4_expectedPose, H4_actualPose, 1e-5));
  CHECK( gtsam::assert_equal(H5_expectedPose, H5_actualPose, 1e-5));

  // Checking for Vel part in the jacobians
  // ******
  Matrix H1_actualVel(H1_actual.block(6,0,3,H1_actual.cols()));
  Matrix H2_actualVel(H2_actual.block(6,0,3,H2_actual.cols()));
  Matrix H3_actualVel(H3_actual.block(6,0,3,H3_actual.cols()));
  Matrix H4_actualVel(H4_actual.block(6,0,3,H4_actual.cols()));
  Matrix H5_actualVel(H5_actual.block(6,0,3,H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  gtsam::Matrix H1_expectedVel, H2_expectedVel, H3_expectedVel, H4_expectedVel, H5_expectedVel;
  H1_expectedVel = gtsam::numericalDerivative11<LieVector, Pose3>(boost::bind(&predictionErrorVel, _1, Vel1, Bias1, Pose2, Vel2, factor), Pose1);
  H2_expectedVel = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionErrorVel, Pose1, _1, Bias1, Pose2, Vel2, factor), Vel1);
  H3_expectedVel = gtsam::numericalDerivative11<LieVector, imuBias::ConstantBias>(boost::bind(&predictionErrorVel, Pose1, Vel1, _1, Pose2, Vel2, factor), Bias1);
  H4_expectedVel = gtsam::numericalDerivative11<LieVector, Pose3>(boost::bind(&predictionErrorVel, Pose1, Vel1, Bias1, _1, Vel2, factor), Pose2);
  H5_expectedVel = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionErrorVel, Pose1, Vel1, Bias1, Pose2, _1, factor), Vel2);

  // Verify they are equal for this choice of state
  CHECK( gtsam::assert_equal(H1_expectedVel, H1_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H2_expectedVel, H2_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H3_expectedVel, H3_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H4_expectedVel, H4_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H5_expectedVel, H5_actualVel, 1e-5));
}




/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ConstructorWithTransform)
{
  gtsam::Key Pose1(11);
  gtsam::Key Pose2(12);
  gtsam::Key Vel1(21);
  gtsam::Key Vel2(22);
  gtsam::Key Bias1(31);

  Vector measurement_acc((Vector(3) << 0.1, 0.2, 0.4));
  Vector measurement_gyro((Vector(3) << -0.2, 0.5, 0.03));

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(0.0, 0.0, 0.0));  // IMU is in ENU orientation


  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, EqualsWithTransform)
{
  gtsam::Key Pose1(11);
  gtsam::Key Pose2(12);
  gtsam::Key Vel1(21);
  gtsam::Key Vel2(22);
  gtsam::Key Bias1(31);

  Vector measurement_acc((Vector(3) << 0.1, 0.2, 0.4));
  Vector measurement_gyro((Vector(3) << -0.2, 0.5, 0.03));

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(0.0, 0.0, 0.0));  // IMU is in ENU orientation


  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);
  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> g(Pose1, Vel1, Bias1, Pose2, Vel2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);
  CHECK(assert_equal(f, g, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, PredictWithTransform)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0));  // IMU is in ENU orientation


  // First test: zero angular motion, some acceleration
  Vector measurement_gyro((Vector(3) << 0.0, 0.0, 0.0));    // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = (Vector(3) << 0.2, 0.1, -0.3+9.81) + omega__cross*omega__cross*body_P_sensor.rotation().inverse().matrix()*body_P_sensor.translation().vector();  // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  LieVector Vel1((Vector(3) << 0.50, -0.50, 0.40));
  imuBias::ConstantBias Bias1;
  Pose3 expectedPose2(Rot3(), Point3(2.05, 0.95, 3.04));
  LieVector expectedVel2((Vector(3) << 0.51, -0.48, 0.43));
  Pose3 actualPose2;
  LieVector actualVel2;
  f.predict(Pose1, Vel1, Bias1, actualPose2, actualVel2);

  CHECK(assert_equal(expectedPose2, actualPose2, 1e-5));
  CHECK(assert_equal(expectedVel2, actualVel2, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ErrorPosVelWithTransform)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0));  // IMU is in ENU orientation


  // First test: zero angular motion, some acceleration
  Vector measurement_gyro((Vector(3) << 0.0, 0.0, 0.0));    // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = (Vector(3) << 0.2, 0.1, -0.3+9.81) + omega__cross*omega__cross*body_P_sensor.rotation().inverse().matrix()*body_P_sensor.translation().vector();  // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);

  Pose3 Pose1(Rot3(), Point3(2.00, 1.00, 3.00));
  Pose3 Pose2(Rot3(), Point3(2.05, 0.95, 3.04));
  LieVector Vel1((Vector(3) << 0.50, -0.50, 0.40));
  LieVector Vel2((Vector(3) << 0.51, -0.48, 0.43));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(zero(9));

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ErrorRotWithTransform)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0));  // IMU is in ENU orientation


  // Second test: zero angular motion, some acceleration
  Vector measurement_gyro((Vector(3) << 0.2, 0.1, -0.3));  // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = (Vector(3) << 0.0, 0.0, 0.0+9.81) + omega__cross*omega__cross*body_P_sensor.rotation().inverse().matrix()*body_P_sensor.translation().vector();  // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);

  Pose3 Pose1(Rot3(), Point3(2.0,1.0,3.0));
  Pose3 Pose2(Rot3::Expmap(body_P_sensor.rotation().matrix()*measurement_gyro*measurement_dt), Point3(2.0, 1.0, 3.0));
  LieVector Vel1((Vector(3) << 0.0,0.0,0.0));
  LieVector Vel2((Vector(3) << 0.0,0.0,0.0));
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(zero(9));

  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST( InertialNavFactor_GlobalVelocity, ErrorRotPosVelWithTransform)
{
  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.1);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0));  // IMU is in ENU orientation


  // Second test: zero angular motion, some acceleration - generated in matlab
  Vector measurement_gyro((Vector(3) << 0.2, 0.1, -0.3)); // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = (Vector(3) << -6.763926150509185,  6.501390843381716,  +2.300389940090343) + omega__cross*omega__cross*body_P_sensor.rotation().inverse().matrix()*body_P_sensor.translation().vector();  // Measured in ENU orientation

  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> f(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);

  Rot3 R1(0.487316618,   0.125253866,   0.86419557,
       0.580273724,  0.693095498, -0.427669306,
      -0.652537293,  0.709880342,  0.265075427);
  Point3 t1(2.0,1.0,3.0);
  Pose3 Pose1(R1, t1);
  LieVector Vel1((Vector(3) << 0.5,-0.5,0.4));
  Rot3 R2(0.473618898,   0.119523052,  0.872582019,
       0.609241153,   0.67099888, -0.422594037,
      -0.636011287,  0.731761397,  0.244979388);
  Point3 t2 = t1.compose( Point3(Vel1*measurement_dt) );
  Pose3 Pose2(R2, t2);
  Vector dv = measurement_dt * (R1.matrix() * body_P_sensor.rotation().matrix() * (Vector(3) << -6.763926150509185,  6.501390843381716,  +2.300389940090343) + world_g);
  LieVector Vel2 = Vel1.compose( dv );
  imuBias::ConstantBias Bias1;

  Vector ActualErr(f.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2));
  Vector ExpectedErr(zero(9));

  // TODO: Expected values need to be updated for global velocity version
  CHECK(assert_equal(ExpectedErr, ActualErr, 1e-5));
}

/* ************************************************************************* */
TEST (InertialNavFactor_GlobalVelocity, JacobianWithTransform ) {

  gtsam::Key PoseKey1(11);
  gtsam::Key PoseKey2(12);
  gtsam::Key VelKey1(21);
  gtsam::Key VelKey2(22);
  gtsam::Key BiasKey1(31);

  double measurement_dt(0.01);
  Vector world_g((Vector(3) << 0.0, 0.0, 9.81));
  Vector world_rho((Vector(3) << 0.0, -1.5724e-05, 0.0)); // NED system
  gtsam::Vector ECEF_omega_earth((Vector(3) << 0.0, 0.0, 7.292115e-5));
  gtsam::Vector world_omega_earth(world_R_ECEF.matrix() * ECEF_omega_earth);

  SharedGaussian model(noiseModel::Isotropic::Sigma(9, 0.1));

  Pose3 body_P_sensor(Rot3(0, 1, 0, 1, 0, 0, 0, 0, -1), Point3(1.0, -2.0, 3.0));  // IMU is in ENU orientation


  Vector measurement_gyro((Vector(3) << 3.14/2, 3.14, +3.14));                                         // Measured in ENU orientation
  Matrix omega__cross = skewSymmetric(measurement_gyro);
  Vector measurement_acc = (Vector(3) << -6.763926150509185,  6.501390843381716,  +2.300389940090343) + omega__cross*omega__cross*body_P_sensor.rotation().inverse().matrix()*body_P_sensor.translation().vector();  // Measured in ENU orientation


  InertialNavFactor_GlobalVelocity<Pose3, LieVector, imuBias::ConstantBias> factor(PoseKey1, VelKey1, BiasKey1, PoseKey2, VelKey2, measurement_acc, measurement_gyro, measurement_dt, world_g, world_rho, world_omega_earth, model, body_P_sensor);

  Rot3 R1(0.487316618,   0.125253866,   0.86419557,
       0.580273724,  0.693095498, -0.427669306,
      -0.652537293,  0.709880342,  0.265075427);
  Point3 t1(2.0,1.0,3.0);
  Pose3 Pose1(R1, t1);
  LieVector Vel1((Vector(3) << 0.5,-0.5,0.4));
  Rot3 R2(0.473618898,   0.119523052,  0.872582019,
       0.609241153,   0.67099888, -0.422594037,
      -0.636011287,  0.731761397,  0.244979388);
  Point3 t2(2.052670960415706,   0.977252139079380,   2.942482135362800);
  Pose3 Pose2(R2, t2);
  LieVector Vel2((Vector(3) << 0.510000000000000,  -0.480000000000000,   0.430000000000000));
  imuBias::ConstantBias Bias1;

  Matrix H1_actual, H2_actual, H3_actual, H4_actual, H5_actual;

  Vector ActualErr(factor.evaluateError(Pose1, Vel1, Bias1, Pose2, Vel2, H1_actual, H2_actual, H3_actual, H4_actual, H5_actual));

  // Checking for Pose part in the jacobians
  // ******
  Matrix H1_actualPose(H1_actual.block(0,0,6,H1_actual.cols()));
  Matrix H2_actualPose(H2_actual.block(0,0,6,H2_actual.cols()));
  Matrix H3_actualPose(H3_actual.block(0,0,6,H3_actual.cols()));
  Matrix H4_actualPose(H4_actual.block(0,0,6,H4_actual.cols()));
  Matrix H5_actualPose(H5_actual.block(0,0,6,H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  gtsam::Matrix H1_expectedPose, H2_expectedPose, H3_expectedPose, H4_expectedPose, H5_expectedPose;
  H1_expectedPose = gtsam::numericalDerivative11<Pose3, Pose3>(boost::bind(&predictionErrorPose, _1, Vel1, Bias1, Pose2, Vel2, factor), Pose1);
  H2_expectedPose = gtsam::numericalDerivative11<Pose3, LieVector>(boost::bind(&predictionErrorPose, Pose1, _1, Bias1, Pose2, Vel2, factor), Vel1);
  H3_expectedPose = gtsam::numericalDerivative11<Pose3, imuBias::ConstantBias>(boost::bind(&predictionErrorPose, Pose1, Vel1, _1, Pose2, Vel2, factor), Bias1);
  H4_expectedPose = gtsam::numericalDerivative11<Pose3, Pose3>(boost::bind(&predictionErrorPose, Pose1, Vel1, Bias1, _1, Vel2, factor), Pose2);
  H5_expectedPose = gtsam::numericalDerivative11<Pose3, LieVector>(boost::bind(&predictionErrorPose, Pose1, Vel1, Bias1, Pose2, _1, factor), Vel2);

  // Verify they are equal for this choice of state
  CHECK( gtsam::assert_equal(H1_expectedPose, H1_actualPose, 1e-5));
  CHECK( gtsam::assert_equal(H2_expectedPose, H2_actualPose, 1e-5));
  CHECK( gtsam::assert_equal(H3_expectedPose, H3_actualPose, 2e-3));
  CHECK( gtsam::assert_equal(H4_expectedPose, H4_actualPose, 1e-5));
  CHECK( gtsam::assert_equal(H5_expectedPose, H5_actualPose, 1e-5));

  // Checking for Vel part in the jacobians
  // ******
  Matrix H1_actualVel(H1_actual.block(6,0,3,H1_actual.cols()));
  Matrix H2_actualVel(H2_actual.block(6,0,3,H2_actual.cols()));
  Matrix H3_actualVel(H3_actual.block(6,0,3,H3_actual.cols()));
  Matrix H4_actualVel(H4_actual.block(6,0,3,H4_actual.cols()));
  Matrix H5_actualVel(H5_actual.block(6,0,3,H5_actual.cols()));

  // Calculate the Jacobian matrices H1 until H5 using the numerical derivative function
  gtsam::Matrix H1_expectedVel, H2_expectedVel, H3_expectedVel, H4_expectedVel, H5_expectedVel;
  H1_expectedVel = gtsam::numericalDerivative11<LieVector, Pose3>(boost::bind(&predictionErrorVel, _1, Vel1, Bias1, Pose2, Vel2, factor), Pose1);
  H2_expectedVel = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionErrorVel, Pose1, _1, Bias1, Pose2, Vel2, factor), Vel1);
  H3_expectedVel = gtsam::numericalDerivative11<LieVector, imuBias::ConstantBias>(boost::bind(&predictionErrorVel, Pose1, Vel1, _1, Pose2, Vel2, factor), Bias1);
  H4_expectedVel = gtsam::numericalDerivative11<LieVector, Pose3>(boost::bind(&predictionErrorVel, Pose1, Vel1, Bias1, _1, Vel2, factor), Pose2);
  H5_expectedVel = gtsam::numericalDerivative11<LieVector, LieVector>(boost::bind(&predictionErrorVel, Pose1, Vel1, Bias1, Pose2, _1, factor), Vel2);

  // Verify they are equal for this choice of state
  CHECK( gtsam::assert_equal(H1_expectedVel, H1_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H2_expectedVel, H2_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H3_expectedVel, H3_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H4_expectedVel, H4_actualVel, 1e-5));
  CHECK( gtsam::assert_equal(H5_expectedVel, H5_actualVel, 1e-5));
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
