/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactor.cpp
 * @brief   Unit test for ImuFactor
 * @author  Luca Carlone, Stephen Williams, Richard Roberts
 */

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
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

ImuFactor::PreintegratedMeasurements evaluatePreintegratedMeasurements(
    const imuBias::ConstantBias& bias,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs,
    const Vector3& initialRotationRate = Vector3(0.0,0.0,0.0)
    )
{
  ImuFactor::PreintegratedMeasurements result(bias, Matrix3::Identity(),
      Matrix3::Identity(), Matrix3::Identity());

  list<Vector3>::const_iterator itAcc = measuredAccs.begin();
  list<Vector3>::const_iterator itOmega = measuredOmegas.begin();
  list<double>::const_iterator itDeltaT = deltaTs.begin();
  for( ; itAcc != measuredAccs.end(); ++itAcc, ++itOmega, ++itDeltaT) {
    result.integrateMeasurement(*itAcc, *itOmega, *itDeltaT);
  }

  return result;
}

Vector3 evaluatePreintegratedMeasurementsPosition(
    const imuBias::ConstantBias& bias,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs,
    const Vector3& initialRotationRate = Vector3(0.0,0.0,0.0) )
{
  return evaluatePreintegratedMeasurements(bias,
      measuredAccs, measuredOmegas, deltaTs, initialRotationRate).deltaPij;
}

Vector3 evaluatePreintegratedMeasurementsVelocity(
    const imuBias::ConstantBias& bias,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs,
    const Vector3& initialRotationRate = Vector3(0.0,0.0,0.0) )
{
  return evaluatePreintegratedMeasurements(bias,
      measuredAccs, measuredOmegas, deltaTs).deltaVij;
}

Rot3 evaluatePreintegratedMeasurementsRotation(
    const imuBias::ConstantBias& bias,
    const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas,
    const list<double>& deltaTs,
    const Vector3& initialRotationRate = Vector3(0.0,0.0,0.0) )
{
  return evaluatePreintegratedMeasurements(bias,
      measuredAccs, measuredOmegas, deltaTs).deltaRij;
}

}

/* ************************************************************************* */
TEST( CombinedImuFactor, PreintegratedMeasurements )
{
  //cout << "++++++++++++++++++++++++++++++ PreintegratedMeasurements +++++++++++++++++++++++++++++++++++++++ " << endl;
  // Linearization point
  imuBias::ConstantBias bias(Vector3(0,0,0), Vector3(0,0,0)); ///< Current estimate of acceleration and angular rate biases

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI/100.0, 0.0, 0.0);
  double deltaT = 0.5;
  double tol = 1e-6;

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements expected1(bias, Matrix3::Zero(),
      Matrix3::Zero(), Matrix3::Zero());
  expected1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedImuFactor::CombinedPreintegratedMeasurements actual1(bias,
      Matrix3::Zero(), Matrix3::Zero(), Matrix3::Zero(),
      Matrix3::Zero(), Matrix3::Zero(), Matrix::Zero(6,6));

//           const imuBias::ConstantBias& bias, ///< Current estimate of acceleration and rotation rate biases
//           const Matrix3& measuredAccCovariance, ///< Covariance matrix of measuredAcc
//           const Matrix3& measuredOmegaCovariance, ///< Covariance matrix of measuredAcc
//           const Matrix3& integrationErrorCovariance, ///< Covariance matrix of measuredAcc
//           const Matrix3& biasAccCovariance, ///< Covariance matrix of biasAcc (random walk describing BIAS evolution)
//           const Matrix3& biasOmegaCovariance, ///< Covariance matrix of biasOmega (random walk describing BIAS evolution)
//           const Matrix& biasAccOmegaInit ///< Covariance of biasAcc & biasOmega when preintegrating measurements

  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expected1.deltaPij), Vector(actual1.deltaPij), tol));
  EXPECT(assert_equal(Vector(expected1.deltaVij), Vector(actual1.deltaVij), tol));
  EXPECT(assert_equal(expected1.deltaRij, actual1.deltaRij, tol));
  DOUBLES_EQUAL(expected1.deltaTij, actual1.deltaTij, tol);
}


/* ************************************************************************* */
TEST( CombinedImuFactor, ErrorWithBiases )
{
  //cout << "++++++++++++++++++++++++++++++ ErrorWithBiases +++++++++++++++++++++++++++++++++++++++ " << endl;

  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)
  imuBias::ConstantBias bias2(Vector3(0.2, 0.2, 0), Vector3(1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI/4.0)), Point3(5.0, 1.0, -50.0));
  LieVector v1((Vector(3) << 0.5, 0.0, 0.0));
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI/4.0 + M_PI/10.0)), Point3(5.5, 1.0, -50.0));
  LieVector v2((Vector(3) << 0.5, 0.0, 0.0));

  // Measurements
  Vector3 gravity; gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis; omegaCoriolis << 0, 0.1, 0.1;
  Vector3 measuredOmega; measuredOmega << 0, 0, M_PI/10.0+0.3;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector() + Vector3(0.2,0.0,0.0);
  double deltaT = 1.0;
  double tol = 1e-6;

  //           const imuBias::ConstantBias& bias, ///< Current estimate of acceleration and rotation rate biases
  //           const Matrix3& measuredAccCovariance, ///< Covariance matrix of measuredAcc
  //           const Matrix3& measuredOmegaCovariance, ///< Covariance matrix of measuredAcc
  //           const Matrix3& integrationErrorCovariance, ///< Covariance matrix of measuredAcc
  //           const Matrix3& biasAccCovariance, ///< Covariance matrix of biasAcc (random walk describing BIAS evolution)
  //           const Matrix3& biasOmegaCovariance, ///< Covariance matrix of biasOmega (random walk describing BIAS evolution)
  //           const Matrix& biasAccOmegaInit ///< Covariance of biasAcc & biasOmega when preintegrating measurements

  Matrix I6x6(6,6);
  I6x6 = Matrix::Identity(6,6);


  ImuFactor::PreintegratedMeasurements pre_int_data(imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
      Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity());

    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

   CombinedImuFactor::CombinedPreintegratedMeasurements Combined_pre_int_data(
       imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0),  Vector3(0.0, 0.0, 0.0)),
        Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity(), 2 * Matrix3::Identity(),  I6x6 );

   Combined_pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);


    // Create factor
    ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity, omegaCoriolis);

    noiseModel::Gaussian::shared_ptr Combinedmodel = noiseModel::Gaussian::Covariance(Combined_pre_int_data.PreintMeasCov);
    CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), Combined_pre_int_data, gravity, omegaCoriolis);


    Vector errorExpected = factor.evaluateError(x1, v1, x2, v2, bias);

    Vector errorActual = Combinedfactor.evaluateError(x1, v1, x2, v2, bias, bias2);


    EXPECT(assert_equal(errorExpected, errorActual.head(9), tol));

    // Expected Jacobians
    Matrix H1e, H2e, H3e, H4e, H5e;
    (void) factor.evaluateError(x1, v1, x2, v2, bias, H1e, H2e, H3e, H4e, H5e);


    // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a, H6a;
  (void) Combinedfactor.evaluateError(x1, v1, x2, v2, bias, bias2, H1a, H2a, H3a, H4a, H5a, H6a);

  EXPECT(assert_equal(H1e, H1a.topRows(9)));
  EXPECT(assert_equal(H2e, H2a.topRows(9)));
  EXPECT(assert_equal(H3e, H3a.topRows(9)));
  EXPECT(assert_equal(H4e, H4a.topRows(9)));
  EXPECT(assert_equal(H5e, H5a.topRows(9)));
}

/* ************************************************************************* */
TEST( CombinedImuFactor, FirstOrderPreIntegratedMeasurements )
{
  //cout << "++++++++++++++++++++++++++++++ FirstOrderPreIntegratedMeasurements +++++++++++++++++++++++++++++++++++++++ " << endl;
  // Linearization point
  imuBias::ConstantBias bias; ///< Current estimate of acceleration and rotation rate biases

  Pose3 body_P_sensor(Rot3::Expmap(Vector3(0,0.1,0.1)), Point3(1, 0, 1));

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
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas, deltaTs, Vector3(M_PI/100.0, 0.0, 0.0));

  // Compute numerical derivatives
  Matrix expectedDelPdelBias = numericalDerivative11<imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsPosition, _1, measuredAccs, measuredOmegas, deltaTs, Vector3(M_PI/100.0, 0.0, 0.0)), bias);
  Matrix expectedDelPdelBiasAcc   = expectedDelPdelBias.leftCols(3);
  Matrix expectedDelPdelBiasOmega = expectedDelPdelBias.rightCols(3);

  Matrix expectedDelVdelBias = numericalDerivative11<imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsVelocity, _1, measuredAccs, measuredOmegas, deltaTs, Vector3(M_PI/100.0, 0.0, 0.0)), bias);
  Matrix expectedDelVdelBiasAcc   = expectedDelVdelBias.leftCols(3);
  Matrix expectedDelVdelBiasOmega = expectedDelVdelBias.rightCols(3);

  Matrix expectedDelRdelBias = numericalDerivative11<Rot3,imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsRotation, _1, measuredAccs, measuredOmegas, deltaTs, Vector3(M_PI/100.0, 0.0, 0.0)), bias);
  Matrix expectedDelRdelBiasAcc   = expectedDelRdelBias.leftCols(3);
  Matrix expectedDelRdelBiasOmega = expectedDelRdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelPdelBiasAcc, preintegrated.delPdelBiasAcc));
  EXPECT(assert_equal(expectedDelPdelBiasOmega, preintegrated.delPdelBiasOmega));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, preintegrated.delVdelBiasAcc));
  EXPECT(assert_equal(expectedDelVdelBiasOmega, preintegrated.delVdelBiasOmega));
  EXPECT(assert_equal(expectedDelRdelBiasAcc, Matrix::Zero(3,3)));
  EXPECT(assert_equal(expectedDelRdelBiasOmega, preintegrated.delRdelBiasOmega, 1e-3)); // 1e-3 needs to be added only when using quaternions for rotations
}

#include <gtsam/linear/GaussianFactorGraph.h>


/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
