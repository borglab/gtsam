/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  NavStateImuExample.cpp
 * @brief Run the NavStateImuEKF on an orbiting (constant twist) scenario.
 *        Uses ScenarioRunner to generate corrupted IMU, de-biases with true
 *        biases, feeds the EKF, and compares to Scenario ground truth.
 *
 * @date   August 2025
 * @authors You
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/NavStateImuEKF.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/navigation/ScenarioRunner.h>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace gtsam;

static constexpr double kGravity = 9.81;

int main(int argc, char* argv[]) {
  // --- Scenario: camera orbiting a point ---
  const double radius = 30.0;
  const double angular_velocity = M_PI;  // rad/sec
  const Vector3 w_b(0, 0, -angular_velocity);
  const Vector3 v_n(radius * angular_velocity, 0, 0);
  ConstantTwistScenario scenario(w_b, v_n);

  // --- Preintegration/IMU parameters ---
  auto params = PreintegrationParams::MakeSharedU(kGravity);
  params->setAccelerometerCovariance(I_3x3 * 0.1);
  params->setGyroscopeCovariance(I_3x3 * 0.1);
  params->setIntegrationCovariance(I_3x3 * 0.1);
  params->setUse2ndOrderCoriolis(false);
  params->setOmegaCoriolis(Vector3::Zero());
  
  // True biases used to corrupt measurements in ScenarioRunner
  imuBias::ConstantBias trueBias(Vector3(0.01, -0.005, 0.02),  // gyro bias
  Vector3(0.05, 0.03, -0.02));  // accel bias
  
  // --- Measurement generator ---
  const double dt = 1.0 / 180.0;         // 1 degree per step
  ScenarioRunner runner(scenario, params, dt, trueBias);

  // --- Initialize EKF with ground truth
  const double t0 = 0.0;
  NavState X0 = scenario.navState(t0);
  Matrix9 P0 = I_9x9 * 1e-2;  // modest initial uncertainty
  NavStateImuEKF ekf(X0, P0, params);

  // --- Run for N steps and compare predictions ---
  const size_t N = 90;
  cout << fixed << setprecision(3);
  cout << "step,t(s),rot_err_deg,pos_err,vel_err\n";
  double t = t0;
  for (size_t i = 0; i < N; ++i) {
    // Measurements from runner, *not* corrupted by noise
    const Vector3 measuredOmega = runner.actualAngularVelocity(t);
    const Vector3 measuredAcc = runner.actualSpecificForce(t);

    // De-bias using the true (known) biases before feeding EKF
    const Vector3 omega = measuredOmega - trueBias.gyroscope();
    const Vector3 acc = measuredAcc - trueBias.accelerometer();

    // Predict one step
    ekf.predict(omega, acc, dt);

    // Ground truth at next time
    t += dt;
    const NavState gt = scenario.navState(t);

    // Print ground truth and EKF state
    // cout << "Ground Truth: " << gt << "\n";
    // cout << "EKF State: " << ekf.state() << "\n";

    // Errors
    const Rot3 dR = gt.attitude().between(ekf.state().attitude());
    const Vector3 rpy = dR.rpy();
    const double rot_err_deg =
        (Vector3(rpy.cwiseAbs()) * (180.0 / M_PI)).norm();
    const double pos_err = (gt.position() - ekf.state().position()).norm();
    const double vel_err = (gt.velocity() - ekf.state().velocity()).norm();

    cout << i + 1 << ", error: " << t << "," << rot_err_deg << "," << pos_err
         << "," << vel_err << "\n";
  }

  return 0;
}
