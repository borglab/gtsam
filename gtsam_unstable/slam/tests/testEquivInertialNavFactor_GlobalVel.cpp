/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testEquivInertialNavFactor_GlobalVel.h.cpp
 * @brief   Unit test for the InertialNavFactor_GlobalVelocity
 * @author  Vadim Indelman, Stephen Williams
 */

#include <gtsam_unstable/slam/EquivInertialNavFactor_GlobalVel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind/bind.hpp>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( EquivInertialNavFactor_GlobalVel, Constructor)
{
  Key poseKey1(11);
  Key poseKey2(12);
  Key velKey1(21);
  Key velKey2(22);
  Key biasKey1(31);

  // IMU accumulation variables
  Vector delta_pos_in_t0 = Vector3(0.0, 0.0, 0.0);
  Vector delta_vel_in_t0 = Vector3(0.0, 0.0, 0.0);
  Vector delta_angles = Vector3(0.0, 0.0, 0.0);
  double delta_t = 0.0;
  Matrix EquivCov_Overall = Matrix::Zero(15,15);
  Matrix Jacobian_wrt_t0_Overall = Matrix::Identity(15,15);
  imuBias::ConstantBias bias1 = imuBias::ConstantBias();

  // Earth Terms (gravity, etc)
  Vector3 g(0.0, 0.0, -9.80);
  Vector3 rho(0.0, 0.0, 0.0);
  Vector3 omega_earth(0.0, 0.0, 0.0);

  // IMU Noise Model
  SharedGaussian imu_model = noiseModel::Gaussian::Covariance(EquivCov_Overall.block(0,0,9,9));

  // Constructor
  EquivInertialNavFactor_GlobalVel<Pose3, Vector3, imuBias::ConstantBias> factor(
      poseKey1, velKey1, biasKey1, poseKey2, velKey2,
          delta_pos_in_t0, delta_vel_in_t0, delta_angles, delta_t,
          g, rho, omega_earth, imu_model, Jacobian_wrt_t0_Overall, bias1);

}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
