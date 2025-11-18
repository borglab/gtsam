/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

 /**
  * @file testManifoldEKF.cpp
  * @brief Unit test for the ManifoldEKF base class using Unit3.
  * @date April 26, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ManifoldEKF.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace gtsam;

// Define simple dynamics for Unit3: constant velocity in the tangent space
namespace exampleUnit3 {

  // Predicts the next state given current state, tangent velocity, and dt
  Unit3 f(const Unit3& p, const Vector2& v, double dt) {
    return p.retract(v * dt);
  }

  // Define a measurement model: measure the z-component of the Unit3 direction
  // H is the Jacobian dh/d(local(p))
  double measureZ(const Unit3& p, OptionalJacobian<1, 2> H) {
    if (H) {
      // H = d(p.point3().z()) / d(local(p))
      // Calculate numerically for simplicity in test
      auto h = [](const Unit3& p_) { return p_.point3().z(); };
      *H = numericalDerivative11<double, Unit3, 2>(h, p);
    }
    return p.point3().z();
  }

} // namespace exampleUnit3

// Test fixture for ManifoldEKF with Unit3
struct Unit3EKFTest {
  Unit3 p0;
  Matrix2 P0;
  Vector2 velocity;
  double dt;
  Matrix2 Q; // Process noise
  Matrix1 R; // Measurement noise

  Unit3EKFTest() :
    p0(Unit3(Point3(1, 0, 0))), // Start pointing along X-axis
    P0(I_2x2 * 0.01),
    velocity((Vector2() << 0.0, M_PI / 4.0).finished()), // Rotate towards +Z axis
    dt(0.1),
    Q(I_2x2 * 0.001),
    R(Matrix1::Identity() * 0.01) {
  }
};


TEST(ManifoldEKF_Unit3, Predict) {
  Unit3EKFTest data;

  // Initialize the EKF
  ManifoldEKF<Unit3> ekf(data.p0, data.P0);

  // --- Prepare inputs for ManifoldEKF::predict ---
  // 1. Compute expected next state
  Unit3 p_next_expected = exampleUnit3::f(data.p0, data.velocity, data.dt);

  // 2. Compute state transition Jacobian F = d(local(p_next)) / d(local(p))
  //    We can compute this numerically using the f function.
  //    GTSAM's numericalDerivative handles derivatives *between* manifolds.
  auto predict_wrapper = [&](const Unit3& p) -> Unit3 {
    return exampleUnit3::f(p, data.velocity, data.dt);
    };
  Matrix2 F = numericalDerivative11<Unit3, Unit3>(predict_wrapper, data.p0);

  // --- Perform EKF prediction ---
  ekf.predict(p_next_expected, F, data.Q);

  // --- Verification ---
  // Check state
  EXPECT(assert_equal(p_next_expected, ekf.state(), 1e-8));

  // Check covariance
  Matrix2 P_expected = F * data.P0 * F.transpose() + data.Q;
  EXPECT(assert_equal(P_expected, ekf.covariance(), 1e-8));

  // Check F manually for a simple case (e.g., zero velocity should give Identity)
  Vector2 zero_velocity = Vector2::Zero();
  auto predict_wrapper_zero = [&](const Unit3& p) -> Unit3 {
    return exampleUnit3::f(p, zero_velocity, data.dt);
    };
  Matrix2 F_zero = numericalDerivative11<Unit3, Unit3>(predict_wrapper_zero, data.p0);
  EXPECT(assert_equal<Matrix2>(I_2x2, F_zero, 1e-8));

}

TEST(ManifoldEKF_Unit3, Update) {
  Unit3EKFTest data;

  // Use a slightly different starting point and covariance for variety
  Unit3 p_start = Unit3(Point3(0, 1, 0)).retract((Vector2() << 0.1, 0).finished()); // Perturb pointing along Y
  Matrix2 P_start = I_2x2 * 0.05;
  ManifoldEKF<Unit3> ekf(p_start, P_start);

  // Simulate a measurement (e.g., true value + noise)
  double z_true = exampleUnit3::measureZ(p_start, {});
  double z_observed = z_true + 0.02; // Add some noise

  // --- Perform EKF update ---
  ekf.update(exampleUnit3::measureZ, z_observed, data.R);

  // --- Verification (Manual Kalman Update Steps) ---
  // 1. Predict measurement and get Jacobian H
  Matrix12 H; // Note: Jacobian is 1x2 for Unit3
  double z_pred = exampleUnit3::measureZ(p_start, H);

  // 2. Innovation and Covariance
  double y = z_pred - z_observed; // Innovation (using vector subtraction for z)
  Matrix1 S = H * P_start * H.transpose() + data.R; // 1x1 matrix

  // 3. Kalman Gain K
  Matrix K = P_start * H.transpose() * S.inverse(); // 2x1 matrix

  // 4. State Correction (in tangent space)
  Vector2 delta_xi = -K * y; // 2x1 vector

  // 5. Expected Updated State and Covariance
  Unit3 p_updated_expected = p_start.retract(delta_xi);
  Matrix2 I_KH = I_2x2 - K * H;
  Matrix2 P_updated_expected = I_KH * P_start;

  // --- Compare EKF result with manual calculation ---
  EXPECT(assert_equal(p_updated_expected, ekf.state(), 1e-8));
  EXPECT(assert_equal(P_updated_expected, ekf.covariance(), 1e-8));
}

// Define simple dynamics and measurement for a 2x2 Matrix state
namespace manifold_ekf_example {

  // Predicts the next state given current state (Matrix), tangent "velocity" (Vector), and dt.
  Matrix f(const Matrix& p, const Vector& vTangent, double dt) {
    return traits<Matrix>::Retract(p, vTangent * dt); // +
  }

  // Define a measurement model: measure the trace of the Matrix (assumed 2x2 here)
  double h(const Matrix& p, OptionalJacobian<-1, -1> H = {}) {
    // Specialized for a 2x2 matrix!
    if (p.rows() != 2 || p.cols() != 2) {
      throw std::invalid_argument("Matrix must be 2x2.");
    }
    if (H) {
      H->resize(1, p.size());
      *H << 1.0, 0.0, 0.0, 1.0; // d(trace)/dp00, d(trace)/dp01, d(trace)/dp10, d(trace)/dp11
    }
    return p(0, 0) + p(1, 1); // Trace of the matrix
  }

} // namespace manifold_ekf_example

TEST(ManifoldEKF_DynamicMatrix, CombinedPredictAndUpdate) {
  Matrix pInitial = (Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  Matrix pInitialCovariance = I_4x4 * 0.01; // Covariance for 2x2 matrix (4x4)
  Vector vTangent = (Vector(4) << 0.5, 0.1, -0.1, -0.5).finished(); // [dp00, dp10, dp01, dp11]/sec
  double deltaTime = 0.1;
  Matrix processNoiseCovariance = I_4x4 * 0.001; // Process noise covariance (4x4)
  Matrix measurementNoiseCovariance = Matrix::Identity(1, 1) * 0.005; // Measurement noise covariance (1x1)

  ManifoldEKF<Matrix> ekf(pInitial, pInitialCovariance);
  // For a 2x2 Matrix, tangent space dimension is 2*2=4.
  EXPECT_LONGS_EQUAL(4, ekf.state().size());
  EXPECT_LONGS_EQUAL(pInitial.rows() * pInitial.cols(), ekf.state().size());

  // Predict Step
  Matrix pPredictedMean = manifold_ekf_example::f(pInitial, vTangent, deltaTime);

  // For this linear prediction model (pNext = pCurrent + V*dt in tangent space),
  // Derivative w.r.t deltaXi is Identity.
  Matrix fJacobian = I_4x4;

  ekf.predict(pPredictedMean, fJacobian, processNoiseCovariance);

  EXPECT(assert_equal(pPredictedMean, ekf.state(), 1e-9));
  Matrix pPredictedCovarianceExpected = fJacobian * pInitialCovariance * fJacobian.transpose() + processNoiseCovariance;
  EXPECT(assert_equal(pPredictedCovarianceExpected, ekf.covariance(), 1e-9));

  // Update Step
  Matrix pCurrentForUpdate = ekf.state();
  Matrix pCurrentCovarianceForUpdate = ekf.covariance();

  // True trace of pCurrentForUpdate (which is pPredictedMean)
  double zTrue = manifold_ekf_example::h(pCurrentForUpdate);
  EXPECT_DOUBLES_EQUAL(5.0, zTrue, 1e-9);
  double zObserved = zTrue - 0.03;

  ekf.update(manifold_ekf_example::h, zObserved, measurementNoiseCovariance);

  // Manual Kalman Update Steps for Verification
  Matrix hJacobian(1, 4); // Measurement Jacobian H (1x4 for 2x2 matrix, trace measurement)
  double zPredictionManual = manifold_ekf_example::h(pCurrentForUpdate, hJacobian);
  Matrix hJacobianExpected = (Matrix(1, 4) << 1.0, 0.0, 0.0, 1.0).finished();
  EXPECT(assert_equal(hJacobianExpected, hJacobian, 1e-9));

  // Innovation: y = zObserved - zPredictionManual (since measurement is double)
  double yInnovation = zObserved - zPredictionManual;
  Matrix innovationCovariance = hJacobian * pCurrentCovarianceForUpdate * hJacobian.transpose() + measurementNoiseCovariance;

  Matrix kalmanGain = pCurrentCovarianceForUpdate * hJacobian.transpose() * innovationCovariance.inverse(); // K is 4x1

  // State Correction (in tangent space of Matrix)
  Vector deltaXiTangent = kalmanGain * yInnovation; // deltaXi is 4x1 Vector

  Matrix pUpdatedManualExpected = traits<Matrix>::Retract(pCurrentForUpdate, deltaXiTangent);
  Matrix pUpdatedCovarianceManualExpected = (I_4x4 - kalmanGain * hJacobian) * pCurrentCovarianceForUpdate;

  EXPECT(assert_equal(pUpdatedManualExpected, ekf.state(), 1e-9));
  EXPECT(assert_equal(pUpdatedCovarianceManualExpected, ekf.covariance(), 1e-9));
}

// Test the non-templated wrapper bridge updateWithVector delegates to
// update<Vector>
TEST(ManifoldEKF_DynamicMatrix, UpdateWithVectorBridge) {
  // State is a 2x2 Matrix manifold (dim=4)
  Matrix X0 = (Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  Matrix P0 = I_4x4 * 0.02;
  ManifoldEKF<Matrix> ekf(X0, P0);

  // Simple linear measurement: z = H * vec(X) with m=2
  // Define H to pick [p00 + p11, p01 - p10]
  Matrix H(2, 4);
  H << 1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 1.0, 0.0;

  // prediction = h(X) at current state
  Vector prediction(2);
  prediction << X0(0, 0) + X0(1, 1), X0(0, 1) - X0(1, 0);

  // Observed z = prediction + noise
  Vector z = prediction + (Vector(2) << 0.1, -0.05).finished();
  Matrix R = Matrix::Identity(2, 2) * 0.01;

  // Run the bridge update
  ekf.updateWithVector(prediction, H, z, R);

  // Manual update to compare
  Vector innovation = z - prediction;           // size 2
  Matrix S = H * P0 * H.transpose() + R;        // 2x2
  Matrix K = P0 * H.transpose() * S.inverse();  // 4x2
  Vector delta = K * innovation;                // 4x1
  Matrix X_expected = traits<Matrix>::Retract(X0, delta);
  Matrix P_expected = (I_4x4 - K * H) * P0 * (I_4x4 - K * H).transpose() +
                      K * R * K.transpose();

  EXPECT(assert_equal(X_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P_expected, ekf.covariance(), 1e-9));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}