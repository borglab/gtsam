/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactor.cpp
 * @brief   Unit test for StereoFactor
 * @author  Chris Beall
 */

#include <gtsam_unstable/slam/ImuFactor.h>
//#include <gtsam/nonlinear/NonlinearEquality.h>
//#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
//#include <gtsam/geometry/StereoCamera.h>
//#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/LieVector.h>
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

/* ************************************************************************* */
namespace {
Vector callEvaluateError(const ImuFactor& factor,
    const Pose3& pose_i, const LieVector& vel_i, const Pose3& pose_j, const LieVector& vel_j,
    const LieVector& bias)
{
  return factor.evaluateError(pose_i, vel_i, pose_j, vel_j, bias);
}

Rot3 evaluateRotationError(const ImuFactor& factor,
    const Pose3& pose_i, const LieVector& vel_i, const Pose3& pose_j, const LieVector& vel_j,
    const LieVector& bias)
{
  return Rot3::Expmap(factor.evaluateError(pose_i, vel_i, pose_j, vel_j, bias).tail(3) ) ;
}

ImuFactor::PreintegratedMeasurements evaluatePreintegratedMeasurements(
    const Vector3& biasAcc, const Vector3& biasOmega,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs)
{
  ImuFactor::PreintegratedMeasurements result(biasAcc, biasOmega, Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity());

  list<Vector3>::const_iterator itAcc = measuredAccs.begin();
  list<Vector3>::const_iterator itOmega = measuredOmegas.begin();
  list<double>::const_iterator itDeltaT = deltaTs.begin();
  for( ; itAcc != measuredAccs.end(); ++itAcc, ++itOmega, ++itDeltaT) {
    result.integrateMeasurement(*itAcc, *itOmega, *itDeltaT);
  }

  return result;
}

Vector3 evaluatePreintegratedMeasurementsPosition(
    const Vector3& biasAcc, const Vector3& biasOmega,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs)
{
  return evaluatePreintegratedMeasurements(biasAcc, biasOmega,
      measuredAccs, measuredOmegas, deltaTs).deltaPij;
}

Vector3 evaluatePreintegratedMeasurementsVelocity(
    const Vector3& biasAcc, const Vector3& biasOmega,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs)
{
  return evaluatePreintegratedMeasurements(biasAcc, biasOmega,
      measuredAccs, measuredOmegas, deltaTs).deltaVij;
}

Rot3 evaluatePreintegratedMeasurementsRotation(
    const Vector3& biasAcc, const Vector3& biasOmega,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs)
{
  return evaluatePreintegratedMeasurements(biasAcc, biasOmega,
      measuredAccs, measuredOmegas, deltaTs).deltaRij;
}

Rot3 evaluateRotation(const Vector3 measuredOmega, const Vector3 biasOmega, const double deltaT)
{
  return Rot3::Expmap((measuredOmega - biasOmega) * deltaT);
}

Vector3 evaluateLogRotation(const Vector3 thetahat, const Vector3 deltatheta)
{
  return Rot3::Logmap( Rot3::Expmap(thetahat).compose( Rot3::Expmap(deltatheta) ) );
}

}

/* ************************************************************************* */
TEST( ImuFactor, PreintegratedMeasurements )
{
  // Linearization point
  Vector3 biasAcc; biasAcc << 0,0,0; ///< Current estimate of acceleration bias
  Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI/100.0, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected preintegrated values
  Vector3 expectedDeltaP1; expectedDeltaP1 << 0.5*0.1*0.5*0.5, 0, 0;
  Vector3 expectedDeltaV1(0.05, 0.0, 0.0);
  Rot3 expectedDeltaR1 = Rot3::RzRyRx(0.5 * M_PI/100.0, 0.0, 0.0);
  double expectedDeltaT1(0.5);

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements actual1(biasAcc, biasOmega, Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero());
  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expectedDeltaP1), Vector(actual1.deltaPij), 1e-6));
  EXPECT(assert_equal(Vector(expectedDeltaV1), Vector(actual1.deltaVij), 1e-6));
  EXPECT(assert_equal(expectedDeltaR1, actual1.deltaRij, 1e-6));
  DOUBLES_EQUAL(expectedDeltaT1, actual1.deltaTij, 1e-6);

  // Integrate again
  Vector3 expectedDeltaP2; expectedDeltaP2 << 0.025 + expectedDeltaP1(0) + 0.5*0.1*0.5*0.5, 0, 0;
  Vector3 expectedDeltaV2 = Vector3(0.05, 0.0, 0.0) + expectedDeltaR1.matrix() * measuredAcc * 0.5;
  Rot3 expectedDeltaR2 = Rot3::RzRyRx(2.0 * 0.5 * M_PI/100.0, 0.0, 0.0);
  double expectedDeltaT2(1);

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements actual2 = actual1;
  actual2.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expectedDeltaP2), Vector(actual2.deltaPij), 1e-6));
  EXPECT(assert_equal(Vector(expectedDeltaV2), Vector(actual2.deltaVij), 1e-6));
  EXPECT(assert_equal(expectedDeltaR2, actual2.deltaRij, 1e-6));
  DOUBLES_EQUAL(expectedDeltaT2, actual2.deltaTij, 1e-6);
}

/* ************************************************************************* */
TEST( ImuFactor, Error )
{
  // Linearization point
  Vector bias(6); bias << 0,0,0,0,0,0; // Bias
  Pose3 x1(Rot3::RzRyRx(M_PI/12.0, M_PI/6.0, M_PI/4.0), Point3(5.0, 1.0, -50.0));
  LieVector v1(3, 0.5, 0.0, 0.0);
  Pose3 x2(Rot3::RzRyRx(M_PI/12.0 + M_PI/100.0, M_PI/6.0, M_PI/4.0), Point3(5.5, 1.0, -50.0));
  LieVector v2(3, 0.5, 0.0, 0.0);

  // Measurements
  Vector3 gravity; gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis; omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega; measuredOmega << M_PI/100, 0, 0;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector();
  double deltaT = 1.0;
  ImuFactor::PreintegratedMeasurements pre_int_data(bias.head(3), bias.tail(3),  Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero());
  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(9, 0.15, 0.15, 0.15, 1.5, 1.5, 1.5, 0.5, 0.5, 0.5));
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity, omegaCoriolis, model);

  Vector errorActual = factor.evaluateError(x1, v1, x2, v2, bias);

  // Expected error
  Vector errorExpected(9); errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Pose3>(
      boost::bind(&callEvaluateError, factor, _1, v1, x2, v2, LieVector(bias)), x1);
  Matrix H2e = numericalDerivative11<LieVector>(
      boost::bind(&callEvaluateError, factor, x1, _1, x2, v2, LieVector(bias)), v1);
  Matrix H3e = numericalDerivative11<Pose3>(
      boost::bind(&callEvaluateError, factor, x1, v1, _1, v2, LieVector(bias)), x2);
  Matrix H4e = numericalDerivative11<LieVector>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, _1, LieVector(bias)), v2);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3,Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, bias), x1);
  Matrix RH3e = numericalDerivative11<Rot3,Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, bias), x2);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a;
  (void) factor.evaluateError(x1, v1, x2, v2, bias, H1a, H2a, H3a, H4a);


  // positions and velocities
  Matrix H1etop6 =  H1e.topRows(6);
  Matrix H1atop6 =  H1a.topRows(6);
  EXPECT(assert_equal(H1etop6, H1atop6));
  // rotations
  EXPECT(assert_equal(RH1e, H1a.bottomRows(3)));  // evaluate only the rotation residue

  EXPECT(assert_equal(H2e, H2a));

  // positions and velocities
  Matrix H3etop6 =  H3e.topRows(6);
  Matrix H3atop6 =  H3a.topRows(6);
  EXPECT(assert_equal(H3etop6, H3atop6));
  // rotations
  EXPECT(assert_equal(RH3e, H3a.bottomRows(3)));  // evaluate only the rotation residue

  EXPECT(assert_equal(H4e, H4a));
}

/* ************************************************************************* */
TEST( ImuFactor, ErrorWithBiases )
{
  // Linearization point
//  Vector bias(6); bias << 0.2, 0, 0, 0.1, 0, 0; // Biases (acc, rot)
//  Pose3 x1(Rot3::RzRyRx(M_PI/12.0, M_PI/6.0, M_PI/4.0), Point3(5.0, 1.0, -50.0));
//  LieVector v1(3, 0.5, 0.0, 0.0);
//  Pose3 x2(Rot3::RzRyRx(M_PI/12.0 + M_PI/10.0, M_PI/6.0, M_PI/4.0), Point3(5.5, 1.0, -50.0));
//  LieVector v2(3, 0.5, 0.0, 0.0);
  Vector bias(6); bias << 0.2, 0, 0, 0, 0, 0.3; // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI/4.0)), Point3(5.0, 1.0, -50.0));
  LieVector v1(3, 0.5, 0.0, 0.0);
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI/4.0 + M_PI/10.0)), Point3(5.5, 1.0, -50.0));
  LieVector v2(3, 0.5, 0.0, 0.0);

  // Measurements
  Vector3 gravity; gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis; omegaCoriolis << 0.1, 0, 0.1;
  Vector3 measuredOmega; measuredOmega << 0, 0, M_PI/10.0+0.3;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector() + Vector3(0.2,0.0,0.0);
  double deltaT = 1.0;

  ImuFactor::PreintegratedMeasurements pre_int_data(bias.head(3), Vector_(3, 0.0, 0.0, 0.0),  Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero());
    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

//  ImuFactor::PreintegratedMeasurements pre_int_data(bias.head(3), bias.tail(3));
//    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

    // Create factor
    noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(9, 0.15, 0.15, 0.15, 1.5, 1.5, 1.5, 0.5, 0.5, 0.5));
    ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity, omegaCoriolis, model);

    SETDEBUG("ImuFactor evaluateError", false);
    Vector errorActual = factor.evaluateError(x1, v1, x2, v2, bias);
    SETDEBUG("ImuFactor evaluateError", false);

    // Expected error
    Vector errorExpected(9); errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

    // Expected Jacobians
    Matrix H1e = numericalDerivative11<Pose3>(
        boost::bind(&callEvaluateError, factor, _1, v1, x2, v2, bias), x1);
    Matrix H2e = numericalDerivative11<LieVector>(
        boost::bind(&callEvaluateError, factor, x1, _1, x2, v2, bias), v1);
    Matrix H3e = numericalDerivative11<Pose3>(
        boost::bind(&callEvaluateError, factor, x1, v1, _1, v2, bias), x2);
    Matrix H4e = numericalDerivative11<LieVector>(
        boost::bind(&callEvaluateError, factor, x1, v1, x2, _1, bias), v2);
    Matrix H5e = numericalDerivative11<LieVector>(
           boost::bind(&callEvaluateError, factor, x1, v1, x2, v2, _1), bias);

    // Check rotation Jacobians
    Matrix RH1e = numericalDerivative11<Rot3,Pose3>(
        boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, bias), x1);
    Matrix RH3e = numericalDerivative11<Rot3,Pose3>(
        boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, bias), x2);
    Matrix RH5e = numericalDerivative11<Rot3,LieVector>(
        boost::bind(&evaluateRotationError, factor, x1, v1, x2, v2, _1), bias);

    // Actual Jacobians
    Matrix H1a, H2a, H3a, H4a, H5a;
    (void) factor.evaluateError(x1, v1, x2, v2, bias, H1a, H2a, H3a, H4a, H5a);

    // positions and velocities
    Matrix H1etop6 =  H1e.topRows(6);
    Matrix H1atop6 =  H1a.topRows(6);
    EXPECT(assert_equal(H1etop6, H1atop6));
    // rotations
    EXPECT(assert_equal(RH1e, H1a.bottomRows(3)));  // evaluate only the rotation residue

    EXPECT(assert_equal(H2e, H2a));

    // positions and velocities
    Matrix H3etop6 =  H3e.topRows(6);
    Matrix H3atop6 =  H3a.topRows(6);
    EXPECT(assert_equal(H3etop6, H3atop6));
    // rotations
    EXPECT(assert_equal(RH3e, H3a.bottomRows(3)));  // evaluate only the rotation residue

    EXPECT(assert_equal(H4e, H4a));

    // positions and velocities
     Matrix H5etop6 =  H5e.topRows(6);
     Matrix H5atop6 =  H5a.topRows(6);
     EXPECT(assert_equal(H5etop6, H5atop6));
     // rotations
    EXPECT(assert_equal(RH5e, H5a.bottomRows(3)));  // evaluate only the rotation residue
}

/* ************************************************************************* */
TEST( ImuFactor, PartialDerivativeExpmap )
{
  // Linearization point
  Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega; measuredOmega << 0.1, 0, 0;
  double deltaT = 0.5;


  // Compute numerical derivatives
  Matrix expectedDelRdelBiasOmega = numericalDerivative11<Rot3, LieVector>(boost::bind(
      &evaluateRotation, measuredOmega, _1, deltaT), biasOmega);

  const Matrix3 Jr = ImuFactor::rightJacobianExpMapSO3((measuredOmega - biasOmega) * deltaT);

   Matrix3  actualdelRdelBiasOmega = - Jr * deltaT; // the delta bias appears with the minus sign

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelRdelBiasOmega, actualdelRdelBiasOmega));

}


/* ************************************************************************* */
TEST( ImuFactor, PartialDerivativeLogmap )
{
  // Linearization point
  Vector3 thetahat; thetahat << 0.1,0.1,0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 deltatheta; deltatheta << 0, 0, 0;


  // Compute numerical derivatives
  Matrix expectedDelFdeltheta = numericalDerivative11<LieVector>(boost::bind(
      &evaluateLogRotation, thetahat, _1), deltatheta);

  const Vector3 x = thetahat; // parametrization of so(3)
  const Matrix3 X = skewSymmetric(x); // element of Lie algebra so(3): X = x^
  double normx = norm_2(x);
  const Matrix3  actualDelFdeltheta = Matrix3::Identity() +
       0.5 * X + (1/(normx*normx) - (1+cos(normx))/(2*normx * sin(normx)) ) * X * X;

//  std::cout << "actualDelFdeltheta" << actualDelFdeltheta << std::endl;
//  std::cout << "expectedDelFdeltheta" << expectedDelFdeltheta << std::endl;

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelFdeltheta, actualDelFdeltheta));

}


/* ************************************************************************* */
TEST( ImuFactor, fistOrderExponential )
{
  // Linearization point
    Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias

    // Measurements
    Vector3 measuredOmega; measuredOmega << 0.1, 0, 0;
    double deltaT = 1.0;

    // change w.r.t. linearization point
    double alpha = 0.0;
    Vector3 deltabiasOmega; deltabiasOmega << alpha,alpha,alpha;


    const Matrix3 Jr = ImuFactor::rightJacobianExpMapSO3((measuredOmega - biasOmega) * deltaT);

     Matrix3  delRdelBiasOmega = - Jr * deltaT; // the delta bias appears with the minus sign

     const Matrix expectedRot = Rot3::Expmap((measuredOmega - biasOmega - deltabiasOmega) * deltaT).matrix();

     const Matrix3 hatRot = Rot3::Expmap((measuredOmega - biasOmega) * deltaT).matrix();
     const Matrix3 actualRot =
         hatRot * Rot3::Expmap(delRdelBiasOmega * deltabiasOmega).matrix();
         //hatRot * (Matrix3::Identity() + skewSymmetric(delRdelBiasOmega * deltabiasOmega));

    // Compare Jacobians
    EXPECT(assert_equal(expectedRot, actualRot));

}

/* ************************************************************************* */
TEST( ImuFactor, FirstOrderPreIntegratedMeasurements )
{
  // Linearization point
  Vector3 biasAcc; biasAcc << 0,0,0; ///< Current estimate of acceleration bias
  Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias

  // Measurements
  list<Vector3> measuredAccs, measuredOmegas;
  list<double> deltaTs;
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI/100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI/100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  for(int i=1;i<100;i++)
  {
  measuredAccs.push_back(Vector3(0.05, 0.09, 0.01));
  measuredOmegas.push_back(Vector3(M_PI/100.0, M_PI/300.0, 2*M_PI/100.0));
  deltaTs.push_back(0.01);
  }

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(biasAcc, biasOmega, measuredAccs, measuredOmegas, deltaTs);

  // Compute numerical derivatives
  Matrix expectedDelPdelBiasAcc = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsPosition,
      _1, biasOmega, measuredAccs, measuredOmegas, deltaTs),
      biasAcc);
  Matrix expectedDelPdelBiasOmega = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsPosition,
      biasAcc, _1, measuredAccs, measuredOmegas, deltaTs),
      biasOmega);
  Matrix expectedDelVdelBiasAcc = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsVelocity,
      _1, biasOmega, measuredAccs, measuredOmegas, deltaTs),
      biasAcc);
  Matrix expectedDelVdelBiasOmega = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsVelocity,
      biasAcc, _1, measuredAccs, measuredOmegas, deltaTs),
      biasOmega);
  Matrix expectedDelRdelBiasAcc = numericalDerivative11<Rot3,LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsRotation,
      _1, biasOmega, measuredAccs, measuredOmegas, deltaTs),
      biasAcc);
  Matrix expectedDelRdelBiasOmega = numericalDerivative11<Rot3,LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsRotation,
      biasAcc, _1, measuredAccs, measuredOmegas, deltaTs),
      biasOmega, 1e-10);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelPdelBiasAcc, preintegrated.delPdelBiasAcc));
  EXPECT(assert_equal(expectedDelPdelBiasOmega, preintegrated.delPdelBiasOmega));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, preintegrated.delVdelBiasAcc));
  EXPECT(assert_equal(expectedDelVdelBiasOmega, preintegrated.delVdelBiasOmega));
  EXPECT(assert_equal(expectedDelRdelBiasAcc, Matrix::Zero(3,3)));
  EXPECT(assert_equal(expectedDelRdelBiasOmega, preintegrated.delRdelBiasOmega, 2e-5));
}


/* ************************************************************************* */
TEST( ImuFactor, MeasurementCovarianceEstimation )
{
  // Linearization point
  Vector3 biasAcc; biasAcc << 0,0,0; ///< Current estimate of acceleration bias
  Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias

  // Measurements
  list<Vector3> measuredAccs, measuredOmegas;
  list<double> deltaTs;
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI/100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI/100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  for(int i=1;i<100;i++)
  {
  measuredAccs.push_back(Vector3(0.05, 0.09, 0.01));
  measuredOmegas.push_back(Vector3(M_PI/100.0, M_PI/300.0, 2*M_PI/100.0));
  deltaTs.push_back(0.01);
  }

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(biasAcc, biasOmega, measuredAccs, measuredOmegas, deltaTs);

  // Compute numerical derivatives
  Matrix expectedDelPdelBiasAcc = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsPosition,
      _1, biasOmega, measuredAccs, measuredOmegas, deltaTs),
      biasAcc);
  Matrix expectedDelPdelBiasOmega = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsPosition,
      biasAcc, _1, measuredAccs, measuredOmegas, deltaTs),
      biasOmega);
  Matrix expectedDelVdelBiasAcc = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsVelocity,
      _1, biasOmega, measuredAccs, measuredOmegas, deltaTs),
      biasAcc);
  Matrix expectedDelVdelBiasOmega = numericalDerivative11<LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsVelocity,
      biasAcc, _1, measuredAccs, measuredOmegas, deltaTs),
      biasOmega);
  Matrix expectedDelRdelBiasAcc = numericalDerivative11<Rot3,LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsRotation,
      _1, biasOmega, measuredAccs, measuredOmegas, deltaTs),
      biasAcc);
  Matrix expectedDelRdelBiasOmega = numericalDerivative11<Rot3,LieVector>(boost::bind(
      &evaluatePreintegratedMeasurementsRotation,
      biasAcc, _1, measuredAccs, measuredOmegas, deltaTs),
      biasOmega, 1e-10);

  Matrix Jexpected(9,9);
  Jexpected <<        //deltaP VS (measuredAcc   measuredOmega   intError)
                      - expectedDelPdelBiasAcc,  - expectedDelPdelBiasOmega, - expectedDelPdelBiasAcc,
                          //deltaV VS (measuredAcc   measuredOmega   intError)
                      - expectedDelVdelBiasAcc,  - expectedDelVdelBiasOmega, Matrix3::Zero(),
                           //deltaR VS (measuredAcc   measuredOmega   intError)
                      expectedDelRdelBiasAcc, -expectedDelRdelBiasOmega, Matrix3::Zero();

  Matrix MeasurementCovarianceexpected(9,9);
  MeasurementCovarianceexpected = Jexpected  * preintegrated.measurementCovariance * Jexpected.transpose();

//  std::cout << "preintegratedMeasurementsCovariance inverse" << preintegrated.preintegratedMeasurementsCovariance.inverse() << std::endl;


  // Compare Jacobians
  EXPECT(assert_equal(MeasurementCovarianceexpected, preintegrated.preintegratedMeasurementsCovariance(), 2e-5));
}

#include <gtsam/linear/GaussianFactorGraph.h>
/* ************************************************************************* */
TEST( ImuFactor, LinearizeTiming)
{
  // Linearization point
  Pose3 x1(Rot3::RzRyRx(M_PI/12.0, M_PI/6.0, M_PI/4.0), Point3(5.0, 1.0, -50.0));
  LieVector v1(3, 0.5, 0.0, 0.0);
  Pose3 x2(Rot3::RzRyRx(M_PI/12.0 + M_PI/100.0, M_PI/6.0, M_PI/4.0), Point3(5.5, 1.0, -50.0));
  LieVector v2(3, 0.5, 0.0, 0.0);
  LieVector bias(6, 0.001, 0.002, 0.008, 0.002, 0.004, 0.012);

  // Pre-integrator
  Vector3 biasAcc; biasAcc << 0, 0, 0.10;
  Vector3 biasOmega; biasOmega << 0, 0, 0.10;
  Vector3 gravity; gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis; omegaCoriolis << 0.0001, 0, 0.01;
  ImuFactor::PreintegratedMeasurements pre_int_data(biasAcc, biasOmega, Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity());

  // Pre-integrate Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI/100.0, 0.0, 0.0);
  double deltaT = 0.5;
  for(size_t i = 0; i < 50; ++i) {
    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  }

  // Create factor
  noiseModel::Base::shared_ptr model = noiseModel::Gaussian::Covariance(pre_int_data.preintegratedMeasurementsCovariance());
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity, omegaCoriolis, model);

  Values values;
  values.insert(X(1), x1);
  values.insert(X(2), x2);
  values.insert(V(1), v1);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  Ordering ordering;
  ordering.push_back(X(1));
  ordering.push_back(V(1));
  ordering.push_back(X(2));
  ordering.push_back(V(2));
  ordering.push_back(B(1));

  GaussianFactorGraph graph;
  gttic_(LinearizeTiming);
  for(size_t i = 0; i < 100000; ++i) {
    GaussianFactor::shared_ptr g = factor.linearize(values, ordering);
    graph.push_back(g);
  }
  gttoc_(LinearizeTiming);
  tictoc_finishedIteration_();
  std::cout << "Linear Error: " << graph.error(values.zeroVectors(ordering)) << std::endl;
  tictoc_print_();
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
