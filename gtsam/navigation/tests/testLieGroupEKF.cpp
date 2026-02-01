/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testGroupEKF.cpp
 * @brief Unit test for LieGroupEKF, as well as dynamics used in Rot3 example.
 * @date April 26, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/LieGroupEKF.h>
#include <gtsam/navigation/NavState.h>

using namespace gtsam;

// Duplicate the dynamics function in GEKF_Rot3Example
namespace exampleSO3 {
static constexpr double k = 0.5;
Vector3 dynamics(const Rot3& X, OptionalJacobian<3, 3> H = {}) {
  // φ = Logmap(R), Dφ = ∂φ/∂δR
  Matrix3 D_phi;
  Vector3 phi = Rot3::Logmap(X, D_phi);
  // zero out yaw
  phi[2] = 0.0;
  D_phi.row(2).setZero();

  if (H) *H = -k * D_phi;  // ∂(–kφ)/∂δR
  return -k * phi;         // xi ∈ 𝔰𝔬(3)
}
}  // namespace exampleSO3

TEST(LieGroupEKF, DynamicsJacobian) {
  // Construct a nontrivial state and IMU input
  Rot3 R = Rot3::RzRyRx(0.1, -0.2, 0.3);

  // Analytic Jacobian
  Matrix3 actualH;
  exampleSO3::dynamics(R, actualH);

  // Numeric Jacobian w.r.t. the state X
  auto f = [&](const Rot3& X_) { return exampleSO3::dynamics(X_); };
  Matrix3 expectedH = numericalDerivative11<Vector3, Rot3>(f, R);

  // Compare
  EXPECT(assert_equal(expectedH, actualH));
}

TEST(LieGroupEKF, PredictNumericState) {
  // GIVEN
  Rot3 R0 = Rot3::RzRyRx(0.2, -0.1, 0.3);
  Matrix3 P0 = Matrix3::Identity() * 0.2;
  double dt = 0.1;

  // Analytic Jacobian
  Matrix3 actualH;
  LieGroupEKF<Rot3> ekf0(R0, P0);
  ekf0.predictMean(exampleSO3::dynamics, dt, actualH);

  // wrap predict into a state->state functor (mapping on SO(3))
  auto g = [&](const Rot3& R) -> Rot3 {
    LieGroupEKF<Rot3> ekf(R, P0);
    return ekf.predictMean(exampleSO3::dynamics, dt);
  };

  // numeric Jacobian of g at R0
  Matrix3 expectedH = numericalDerivative11<Rot3, Rot3>(g, R0);

  EXPECT(assert_equal(expectedH, actualH));
}

TEST(LieGroupEKF, Rot3HigherOrderTransition) {
  Rot3 R0 = Rot3::RzRyRx(0.2, -0.1, 0.3);
  Matrix3 P0 = Matrix3::Identity() * 0.2;
  double dt = 0.1;

  Matrix3 PhiK1, PhiK2, PhiHighOrder;

  {
    LieGroupEKF<Rot3> ekf(R0, P0);
    ekf.predictMean<1>(exampleSO3::dynamics, dt, PhiK1);
  }

  {
    LieGroupEKF<Rot3> ekf(R0, P0);
    ekf.predictMean<2>(exampleSO3::dynamics, dt, PhiK2);
  }

  {
    LieGroupEKF<Rot3> ekf(R0, P0);
    ekf.predictMean<6>(exampleSO3::dynamics, dt, PhiHighOrder);
  }

  double diffK1 = (PhiK1 - PhiHighOrder).norm();
  double diffK2 = (PhiK2 - PhiHighOrder).norm();

  EXPECT(diffK2 < diffK1);
  EXPECT((PhiK1 - PhiK2).norm() > 1e-6);
}

TEST(LieGroupEKF, StateAndControl) {
  auto f = [](const Rot3& X, const Vector2& dummy_u,
              OptionalJacobian<3, 3> H = {}) {
    return exampleSO3::dynamics(X, H);
  };

  // GIVEN
  Rot3 R0 = Rot3::RzRyRx(0.2, -0.1, 0.3);
  Matrix3 P0 = Matrix3::Identity() * 0.2;
  Vector2 dummy_u(1, 2);
  double dt = 0.1;
  Matrix3 Q = Matrix3::Zero();

  // Analytic Jacobian
  Matrix3 actualH;
  LieGroupEKF<Rot3> ekf0(R0, P0);
  ekf0.predictMean(f, dummy_u, dt, actualH);

  // wrap predict into a state->state functor (mapping on SO(3))
  auto g = [&](const Rot3& R) -> Rot3 {
    LieGroupEKF<Rot3> ekf(R, P0);
    return ekf.predictMean(f, dummy_u, dt, Q);
  };

  // numeric Jacobian of g at R0
  Matrix3 expectedH = numericalDerivative11<Rot3, Rot3>(g, R0);

  EXPECT(assert_equal(expectedH, actualH));
}

// Namespace for dynamic Matrix LieGroupEKF test
namespace exampleLieGroupDynamicMatrix {
// Constant tangent vector for dynamics (same as "velocityTangent" in IEKF test)
const Vector kFixedVelocityTangent =
    (Vector(4) << 0.5, 0.1, -0.1, -0.5).finished();

// Dynamics function: xi = f(X, H_X)
// Returns a constant tangent vector, so Df_DX = 0.
// H_X is D(xi)/D(X_local), where X_local is the tangent space perturbation of
// X.
Vector f(const Matrix& X,
         OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_X = {}) {
  if (H_X) {
    size_t state_dim = X.size();
    size_t tangent_dim = kFixedVelocityTangent.size();
    // Ensure Jacobian dimensions are consistent even if state or tangent is
    // 0-dim
    H_X->setZero(tangent_dim, state_dim);
  }
  return kFixedVelocityTangent;
}

// Measurement function h(X, H)
double h(const Matrix& p, OptionalJacobian<-1, -1> H = {}) {
  // Specialized for a 2x2 matrix!
  if (p.rows() != 2 || p.cols() != 2) {
    throw std::invalid_argument("Matrix must be 2x2.");
  }
  if (H) {
    H->resize(1, p.size());
    *H << 1.0, 0.0, 0.0,
        1.0;  // d(trace)/dp00, d(trace)/dp01, d(trace)/dp10, d(trace)/dp11
  }
  return p(0, 0) + p(1, 1);  // Trace of the matrix
}
}  // namespace exampleLieGroupDynamicMatrix

TEST(LieGroupEKF_DynamicMatrix, PredictAndUpdate) {
  // --- Setup ---
  Matrix p0Matrix = (Matrix(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  Matrix p0Covariance = I_4x4 * 0.01;
  double dt = 0.1;
  // Continuous-time.
  Matrix process_noise_Qc = I_4x4 * 0.01;
  Matrix measurement_noise_R = Matrix::Identity(1, 1) * 0.005;

  LieGroupEKF<Matrix> ekf(p0Matrix, p0Covariance);
  EXPECT_LONGS_EQUAL(4, ekf.state().size());
  EXPECT_LONGS_EQUAL(4, ekf.dimension());

  // --- Predict ---
  // ekf.predict takes f(X, H_X), dt, process_noise_Q
  ekf.predict(exampleLieGroupDynamicMatrix::f, dt, process_noise_Qc);

  // Verification for Predict
  // For f, Df_DXk = 0 (Jacobian of xi w.r.t X_local is Zero).
  // State transition Jacobian A = Ad_Uinv + Dexp * Df_DXk * dt.
  // For Matrix (VectorSpace): Ad_Uinv = I, Dexp = I.
  // So, A = I + I * 0 * dt = I.
  // Covariance update: P_next = A * P_current * A.transpose() + Q*dt = I *
  // P_current * I + Q*dt = P_current + Q*dt.
  Matrix pPredictedExpected = traits<Matrix>::Retract(
      p0Matrix, exampleLieGroupDynamicMatrix::kFixedVelocityTangent * dt);
  Matrix pCovariancePredictedExpected = p0Covariance + process_noise_Qc * dt;

  EXPECT(assert_equal(pPredictedExpected, ekf.state(), 1e-9));
  EXPECT(assert_equal(pCovariancePredictedExpected, ekf.covariance(), 1e-9));

  // --- Update ---
  Matrix pStateBeforeUpdate = ekf.state();
  Matrix pCovarianceBeforeUpdate = ekf.covariance();

  double zTrue = exampleLieGroupDynamicMatrix::h(pStateBeforeUpdate);
  double zObserved = zTrue - 0.03;  // Simulated measurement with some error

  ekf.update(exampleLieGroupDynamicMatrix::h, zObserved, measurement_noise_R);

  // Verification for Update (Manual Kalman Steps)
  Matrix H_update(
      1, 4);  // Measurement Jacobian: 1x4 for 2x2 matrix, trace measurement
  double zPredictionManual =
      exampleLieGroupDynamicMatrix::h(pStateBeforeUpdate, H_update);
  // Innovation: y_tangent = traits<Measurement>::Local(prediction, observation)
  // For double (scalar), Local(A,B) is B-A.
  double innovationY_tangent = zObserved - zPredictionManual;
  Matrix S_innovation_cov =
      H_update * pCovarianceBeforeUpdate * H_update.transpose() +
      measurement_noise_R;
  Matrix K_gain = pCovarianceBeforeUpdate * H_update.transpose() *
                  S_innovation_cov.inverse();
  Vector deltaXiTangent =
      K_gain *
      innovationY_tangent;  // Tangent space correction for Matrix state
  Matrix pUpdatedExpected =
      traits<Matrix>::Retract(pStateBeforeUpdate, deltaXiTangent);
  Matrix I_KH =
      I_4x4 - K_gain * H_update;  // I_4x4 because state dimension is 4
  Matrix pUpdatedCovarianceExpected =
      I_KH * pCovarianceBeforeUpdate * I_KH.transpose() +
      K_gain * measurement_noise_R * K_gain.transpose();

  EXPECT(assert_equal(pUpdatedExpected, ekf.state(), 1e-9));
  EXPECT(assert_equal(pUpdatedCovarianceExpected, ekf.covariance(), 1e-9));
}

namespace matrixK1Consistency {
struct LinearDynamics {
  Matrix A;
  Vector xi;
  LinearDynamics() {
    A = (Matrix(4, 4) << 0.1, -0.2, 0.0, 0.3,  //
         0.05, 0.1, -0.15, 0.2,                //
         -0.25, 0.0, 0.05, -0.1,               //
         0.0, 0.2, -0.05, 0.15)
            .finished();
    xi = (Vector(4) << 0.4, -0.3, 0.2, 0.1).finished();
  }

  Vector operator()(
      const Matrix&,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) const {
    if (H) *H = A;
    return xi;
  }
};
}  // namespace matrixK1Consistency

TEST(LieGroupEKF_DynamicMatrix, FirstOrderMatchesExpmK1) {
  Matrix X0 = Matrix::Zero(2, 2);
  Matrix P0 = I_4x4 * 0.1;
  double dt = 0.05;
  LieGroupEKF<Matrix> ekf(X0, P0);

  matrixK1Consistency::LinearDynamics dynamics;
  Matrix actualA_storage = Matrix::Zero(4, 4);
  OptionalJacobian<-1, -1> actualA(actualA_storage);
  ekf.predictMean<1>(dynamics, dt, actualA);

  Matrix expectedA = expm(dynamics.A * dt, 1);
  EXPECT(assert_equal(expectedA, actualA_storage));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
