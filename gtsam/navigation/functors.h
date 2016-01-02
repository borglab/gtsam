/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    functors.h
 * @brief   Functors for use in Navigation factors
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <cmath>

namespace gtsam {

// Implement Rot3::ExpmapDerivative(omega) * theta, with derivatives
static Vector3 correctWithExpmapDerivative(
    const Vector3& omega, const Vector3& theta,
    OptionalJacobian<3, 3> H1 = boost::none,
    OptionalJacobian<3, 3> H2 = boost::none) {
  using std::sin;
  const double angle2 = omega.dot(omega);  // rotation angle, squared
  if (angle2 <= std::numeric_limits<double>::epsilon()) {
    if (H1) *H1 = 0.5 * skewSymmetric(theta);
    if (H2) *H2 = I_3x3;
    return theta;
  }

  const double angle = std::sqrt(angle2);  // rotation angle
  const double s1 = sin(angle) / angle;
  const double s2 = sin(angle / 2.0);
  const double a = 2.0 * s2 * s2 / angle2;
  const double b = (1.0 - s1) / angle2;

  const Vector3 omega_x_theta = omega.cross(theta);
  const Vector3 yt = a * omega_x_theta;

  const Matrix3 W = skewSymmetric(omega);
  const Vector3 omega_x_omega_x_theta = omega.cross(omega_x_theta);
  const Vector3 yyt = b * omega_x_omega_x_theta;

  if (H1) {
    Matrix13 omega_t = omega.transpose();
    const Matrix3 T = skewSymmetric(theta);
    const double Da = (s1 - 2.0 * a) / angle2;
    const double Db = (3.0 * s1 - cos(angle) - 2.0) / angle2 / angle2;
    *H1 = (-Da * omega_x_theta + Db * omega_x_omega_x_theta) * omega_t + a * T -
          b * skewSymmetric(omega_x_theta) - b * W * T;
  }
  if (H2) *H2 = I_3x3 - a* W + b* W* W;

  return theta - yt + yyt;
}

// theta(k+1) = theta(k) + inverse(H)*omega dt
// omega = (H/dt_)*(theta(k+1) - H*theta(k))
// TODO(frank): make linear expression
class PredictAngularVelocity {
 private:
  double dt_;

 public:
  typedef Vector3 result_type;

  PredictAngularVelocity(double dt) : dt_(dt) {}

  Vector3 operator()(const Vector3& theta, const Vector3& theta_plus,
                     OptionalJacobian<3, 3> H1 = boost::none,
                     OptionalJacobian<3, 3> H2 = boost::none) const {
    // TODO(frank): take into account derivative of ExpmapDerivative
    const Vector3 predicted = (theta_plus - theta) / dt_;
    Matrix3 D_c_t, D_c_p;
    const Vector3 corrected =
        correctWithExpmapDerivative(theta, predicted, D_c_t, D_c_p);
    if (H1) *H1 = D_c_t - D_c_p / dt_;
    if (H2) *H2 = D_c_p / dt_;
    return corrected;
  }
};

// TODO(frank): make linear expression
static Vector3 averageVelocity(const Vector3& vel, const Vector3& vel_plus,
                               OptionalJacobian<3, 3> H1 = boost::none,
                               OptionalJacobian<3, 3> H2 = boost::none) {
  // TODO(frank): take into account derivative of ExpmapDerivative
  if (H1) *H1 = 0.5 * I_3x3;
  if (H2) *H2 = 0.5 * I_3x3;
  return 0.5 * (vel + vel_plus);
}

// pos(k+1) = pos(k) + average_velocity * dt
// TODO(frank): make linear expression
class PositionDefect {
 private:
  double dt_;

 public:
  typedef Vector3 result_type;

  PositionDefect(double dt) : dt_(dt) {}

  Vector3 operator()(const Vector3& pos, const Vector3& pos_plus,
                     const Vector3& average_velocity,
                     OptionalJacobian<3, 3> H1 = boost::none,
                     OptionalJacobian<3, 3> H2 = boost::none,
                     OptionalJacobian<3, 3> H3 = boost::none) const {
    // TODO(frank): take into account derivative of ExpmapDerivative
    if (H1) *H1 = I_3x3;
    if (H2) *H2 = -I_3x3;
    if (H3) *H3 = I_3x3* dt_;
    return (pos + average_velocity * dt_) - pos_plus;
  }
};

// vel(k+1) = vel(k) + Rk * acc * dt
// acc = Rkt * [vel(k+1) - vel(k)]/dt
// TODO(frank): take in Rot3
class PredictAcceleration {
 private:
  double dt_;

 public:
  typedef Vector3 result_type;

  PredictAcceleration(double dt) : dt_(dt) {}

  Vector3 operator()(const Vector3& vel, const Vector3& vel_plus,
                     const Vector3& theta,
                     OptionalJacobian<3, 3> H1 = boost::none,
                     OptionalJacobian<3, 3> H2 = boost::none,
                     OptionalJacobian<3, 3> H3 = boost::none) const {
    Matrix3 D_R_theta;
    // TODO(frank): D_R_theta is ExpmapDerivative (computed again)
    Rot3 nRb = Rot3::Expmap(theta, D_R_theta);
    Vector3 n_acc = (vel_plus - vel) / dt_;
    Matrix3 D_b_R, D_b_n;
    Vector3 b_acc = nRb.unrotate(n_acc, D_b_R, D_b_n);
    if (H1) *H1 = -D_b_n / dt_;
    if (H2) *H2 = D_b_n / dt_;
    if (H3) *H3 = D_b_R* D_R_theta;
    return b_acc;
  }
};

}  // namespace gtsam
