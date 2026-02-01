/**
 * @file ABCEquivariantFilter.h
 * @brief Attitude-Bias-Calibration Equivariant Filter for state estimation
 * @author Rohan Bansal
 * @date 2026
 *
 * This file extends the EquivariantFilter class to provide a more user-friendly
 * interface for the ABC Equivariant Filter. This class templates on the number
 * of calibrated sensors N, abstracting away the details of the ABC system.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/EquivariantFilter.h>
#include <gtsam_unstable/geometry/ABC.h>

namespace gtsam {
namespace abc {

/**
 * @class AbcEquivariantFilter
 * @brief Equivariant Filter for Attitude-Bias-Calibration (ABC) estimation
 *
 * This class implements an equivariant filter for estimating:
 * - Attitude (rotation): The orientation of the body frame relative to a
 *   reference frame
 * - Bias: Gyroscope bias correction vector (3D)
 * - Calibration: N sensor calibration rotation matrices
 *
 * The filter uses the ABC Lie group structure and equivariant dynamics to
 * provide consistent state estimation. It inherits from EquivariantFilter
 * and provides a simplified interface for ABC-specific operations.
 *
 * @tparam N Number of calibrated sensors (typically 1 or more)
 */
template <size_t N>
class AbcEquivariantFilter
    : public gtsam::EquivariantFilter<State<N>, Symmetry<N>> {
 public:
  using gtsam::EquivariantFilter<State<N>, Symmetry<N>>::update;

  /**
   * @brief Default constructor with identity initial covariance
   *
   * Initializes the filter with an identity state and identity covariance
   * matrix of dimension (6 + 3*N) x (6 + 3*N), where 6 accounts for the
   * attitude and bias parameters, and 3*N for N calibration parameters.
   */
  AbcEquivariantFilter()
      : AbcEquivariantFilter(Matrix::Identity(6 + 3 * N, 6 + 3 * N)) {}

  /**
   * @brief Construct filter with custom initial covariance
   *
   * Initializes the filter at the identity state with a specified initial
   * covariance matrix on the manifold tangent space.
   *
   * @param Sigma0 Initial covariance matrix (6+3*N) x (6+3*N)
   */
  explicit AbcEquivariantFilter(const Matrix& Sigma0)
      : gtsam::EquivariantFilter<State<N>, Symmetry<N>>(State<N>::identity(),
                                                        Sigma0) {}

  /**
   * @brief Prediction step using gyroscope measurements
   *
   * Propagates the filter state forward in time using angular velocity
   * measurements and process noise. This method uses the explicit Jacobian
   * matrices (A and B) computed from the ABC dynamics.
   *
   * @param omega Angular velocity measurement (body frame, rad/s)
   * @param inputCovariance Process noise covariance (6x6) for the input
   * @param dt Time step (seconds)
   */
  void predict(const Vector3& omega, const Matrix6& inputCovariance,
               double dt) {
    const Matrix Q = inputProcessNoise<N>(inputCovariance);
    const Vector6 u = toInputVector(omega);
    const Lift<N> lift_u(u);
    const typename InputAction<N>::Orbit psi_u(u);

    const Group<N> X_hat = this->groupEstimate();
    const Matrix A = stateMatrixA<N>(psi_u, X_hat);
    const Matrix B = inputMatrixB<N>(X_hat);
    const Matrix Qc = B * Q * B.transpose();

    this->template predictWithJacobian<2>(lift_u, A, Qc, dt);
  }

  /**
   * @brief Measurement update using a direction observation
   *
   * Corrects the filter estimate using a direction measurement from a
   * calibrated sensor. The measurement model assumes the sensor observes
   * a known reference direction in the body frame.
   *
   * @param y Measured direction (unit vector in body frame)
   * @param d Reference direction (unit vector in reference frame)
   * @param R Measurement noise covariance (3x3)
   * @param cal_idx Index of the calibrated sensor (0 to N-1), or -1 for
   *                uncalibrated measurements
   */
  void update(const Unit3& y, const Unit3& d, const Matrix3& R, int cal_idx) {
    const Innovation<N> innovation(y, d, cal_idx);
    const Group<N> X_hat = this->groupEstimate();
    const Matrix3 D = outputMatrixD<N>(X_hat, cal_idx);
    const Matrix3 R_adjusted = D * R * D.transpose();
    this->template update<Vector3>(innovation, Z_3x1, R_adjusted);
  }

  /**
   * @brief Get current attitude estimate
   * @return Current rotation estimate (body frame to reference frame)
   */
  Rot3 attitude() const { return this->state().R; }

  /**
   * @brief Get current gyroscope bias estimate
   * @return Current bias vector (rad/s)
   */
  Vector3 bias() const { return this->state().b; }

  /**
   * @brief Get calibration estimate for a specific sensor
   * @param i Sensor index (0 to N-1)
   * @return Calibration rotation for sensor i
   */
  Rot3 calibration(size_t i) const { return this->state().S[i]; }
};

}  // namespace abc
}  // namespace gtsam
