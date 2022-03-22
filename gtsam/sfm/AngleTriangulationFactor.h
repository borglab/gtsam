/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   AngleTriangulationFactor.h
 * @date   December 2021
 * @author Varun Agrawal
 * @brief  Factor for performing constraining triangulation. Uses triangulation
 * as per https://arxiv.org/pdf/1903.09115.pdf.
 */

#pragma once

#include <gtsam/geometry/Cal3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

template <typename CALIBRATION>
class GTSAM_EXPORT AngleTriangulationFactor
    : public NoiseModelFactor2<Pose3, Pose3> {
  using Base = NoiseModelFactor2<Pose3, Pose3>;

 public:
  enum MinimizationType { L1, L2, Linfinity };

 private:
  CALIBRATION K_;
  Point2 u0_, u1_;

  MinimizationType minimizationType_;

 public:
  /// shorthand for this class
  using This = AngleTriangulationFactor;

  /// shorthand for a smart pointer to a factor
  using shared_ptr = boost::shared_ptr<This>;

  AngleTriangulationFactor() {}

  AngleTriangulationFactor(Key j1, Key j2, const CALIBRATION& K,
                           const Point2& u0, const Point2& u1,
                           const SharedNoiseModel& noiseModel,
                           const MinimizationType& minimizationType)
      : Base(noiseModel, j1, j2),
        K_(K),
        u0_(u0),
        u1_(u1),
        minimizationType_(minimizationType) {}

  double vectorAngle(const Vector3& a, const Vector3& b,
                     OptionalJacobian<1, 3> Ha = boost::none,
                     OptionalJacobian<1, 3> Hb = boost::none) const {
    Matrix13 H_dot_a, H_dot_b, H_a_norm, H_b_norm;
    double a_norm = norm3(a, H_a_norm);
    double b_norm = norm3(b, H_b_norm);
    double a_b_norm = a_norm * b_norm;
    double a_b_dot = dot(a, b, H_dot_a, H_dot_b);
    double cos_theta = a_b_dot / (a_b_norm);

    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double theta = acos(cos_theta);

    if (Ha) {
      // If lines are parallel, set jacobian to 0
      if (cos_theta == 1) {
        *Ha = Matrix13::Zero();
      } else {
        Matrix13 H_cos_a = (H_dot_a / a_b_norm) - (a_b_dot * b_norm * H_a_norm /
                                                   (a_b_norm * a_b_norm));
        *Ha = (-1 / sqrt(1 - cos_theta * cos_theta)) * H_cos_a;
      }
    }
    if (Hb) {
      // If lines are parallel, set jacobian to 0
      if (cos_theta == 1) {
        *Hb = Matrix13::Zero();
      } else {
        Matrix13 H_cos_b = (H_dot_b / a_b_norm) - (a_b_dot * a_norm * H_b_norm /
                                                   (a_b_norm * a_b_norm));
        *Hb = (-1 / sqrt(1 - cos_theta * cos_theta)) * H_cos_b;
      }
    }
    return theta;
  }

  std::vector<Vector3> l1TriangulationError(
      const Point3& t, const Vector3& m0, const Vector3& m1,
      const Unit3& m0_hat, const Unit3& m1_hat,
      OptionalJacobian<6, 3> H_t = boost::none,
      OptionalJacobian<6, 3> H_m0 = boost::none,
      OptionalJacobian<6, 3> H_m1 = boost::none) const {
    Vector3 m0_prime, m1_prime;

    if (m0_hat.cross(t).norm() <= m1_hat.cross(t).norm()) {
      Matrix3 H_cross_m1, H_cross_t;
      Matrix23 H_n1_hat;
      Matrix32 H_n1_hat_point;
      Matrix13 H_dot_m0, H_dot_n1_hat;

      // Use equation 12
      Vector n1 = cross(m1, t, H_cross_m1, H_cross_t);
      Vector3 n1_hat = Unit3::FromPoint3(n1, H_n1_hat).point3(H_n1_hat_point);

      double d = dot(m0, n1_hat, H_dot_m0, H_dot_n1_hat);
      m0_prime = m0 - (d * n1_hat);
      m1_prime = m1;

      if (H_t) {
        Matrix3 H_m0_prime_t = -((n1_hat * H_dot_n1_hat * H_n1_hat_point *
                                  H_n1_hat * H_cross_t) +  //
                                 (d * H_n1_hat_point * H_n1_hat * H_cross_t));
        Matrix3 H_m1_prime_t = Matrix3::Zero();

        (*H_t) << H_m0_prime_t, H_m1_prime_t;
      }
      if (H_m0) {
        Matrix3 H_m0_prime_m0 = Matrix3::Identity() - (n1_hat * H_dot_m0);
        Matrix3 H_m1_prime_m0 = Matrix3::Zero();
        (*H_m0) << H_m0_prime_m0, H_m1_prime_m0;
      }
      if (H_m1) {
        Matrix3 H_m1_prime_m0 = -((n1_hat * H_dot_n1_hat * H_n1_hat_point *
                                   H_n1_hat * H_cross_m1) +  //
                                  (d * H_n1_hat_point * H_n1_hat * H_cross_m1));
        Matrix3 H_m1_prime_m1 = Matrix3::Identity();
        (*H_m1) << H_m1_prime_m0, H_m1_prime_m1;
      }

    } else {
      Matrix3 H_cross_m0, H_cross_t;
      Matrix23 H_n0_hat;
      Matrix32 H_n0_hat_point;
      Matrix13 H_m1_dot, H_n0_hat_dot;

      // Use equation 13
      Vector n0 = cross(m0, t, H_cross_m0, H_cross_t);
      Unit3 n0_hat = Unit3::FromPoint3(n0, H_n0_hat);

      m0_prime = m0;

      Vector3 n0_hat_point = n0_hat.point3(H_n0_hat_point);
      double d = dot(m1, n0_hat_point, H_m1_dot, H_n0_hat_dot);
      m1_prime = m1 - (d * n0_hat_point);

      if (H_t) {
        Matrix3 H_m0_prime_t = Matrix3::Zero();
        Matrix3 H_m1_prime_t = -((n0_hat_point * H_n0_hat_dot * H_n0_hat_point *
                                  H_n0_hat * H_cross_t) +  //
                                 (d * H_n0_hat_point * H_n0_hat * H_cross_t));

        (*H_t) << H_m0_prime_t, H_m1_prime_t;
      }
      if (H_m0) {
        Matrix3 H_m0_prime_m0 = Matrix3::Identity();
        Matrix3 H_m1_prime_m0 = -((n0_hat_point * H_n0_hat_dot *
                                   H_n0_hat_point * H_n0_hat * H_cross_m0) +
                                  d * H_n0_hat_point * H_n0_hat * H_cross_m0);
        (*H_m0) << H_m0_prime_m0, H_m1_prime_m0;
      }
      if (H_m1) {
        Matrix3 H_m0_prime_m1 = Matrix3::Zero();
        Matrix3 H_m1_prime_m1 = Matrix3::Identity() - (n0_hat_point * H_m1_dot);
        (*H_m1) << H_m0_prime_m1, H_m1_prime_m1;
      }
    }

    return {m0_prime, m1_prime};
  }

  std::vector<Vector3> l2TriangulationError(
      const Point3& t, const Vector3& m0, const Vector3& m1,
      const Unit3& m0_hat, const Unit3& m1_hat,
      OptionalJacobian<6, 3> H_t = boost::none,
      OptionalJacobian<6, 3> H_m0 = boost::none,
      OptionalJacobian<6, 3> H_m1 = boost::none) const {
    Point3 t_hat = Unit3(t).point3();

    Matrix23 A;
    A.block<1, 3>(0, 0) = m0_hat.unitVector();
    A.block<1, 3>(1, 0) = m1_hat.unitVector();

    Matrix U, V;
    Vector S;

    svd(A * (I_3x3 - (t_hat * t_hat.transpose())), U, S, V);

    Vector3 n_hat_prime = V.block<3, 1>(0, 1);
    Vector3 m0_prime = m0 - (m0.dot(n_hat_prime) * n_hat_prime);
    Vector3 m1_prime = m1 - (m1.dot(n_hat_prime) * n_hat_prime);

    return {m0_prime, m1_prime};
  }

  std::vector<Vector3> lInfinityTriangulationError(
      const Point3& t, const Vector3& m0, const Vector3& m1,
      const Unit3& m0_hat, const Unit3& m1_hat,
      OptionalJacobian<6, 3> H_t = boost::none,
      OptionalJacobian<6, 3> H_m0 = boost::none,
      OptionalJacobian<6, 3> H_m1 = boost::none) const {
    Vector3 n_a = (m0_hat.unitVector() + m1_hat.unitVector()).cross(t);
    Vector3 n_b = (m0_hat.unitVector() - m1_hat.unitVector()).cross(t);

    Vector3 n_prime = n_a.norm() >= n_b.norm() ? n_a : n_b;
    Vector3 n_hat_prime = Unit3(n_prime).unitVector();
    Vector3 m0_prime = m0 - (m0.dot(n_hat_prime) * n_hat_prime);
    Vector3 m1_prime = m1 - (m1.dot(n_hat_prime) * n_hat_prime);

    return {m0_prime, m1_prime};
  }

  Vector evaluateError(
      const Pose3& wTc0, const Pose3& wTc1,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const override {
    Matrix6 H_1_inverse, H_T_1, H_T_2;
    Matrix36 H_R_T, H_t_T;
    Matrix3 H_m0_R;
    Matrix63 H_t, H_m0, H_m1;
    Matrix3 H_Rf0_R;
    Matrix13 H_theta0_Rf0, H_theta0_m0_prime, H_theta1_f1, H_theta1_m1_prime;

    // Get the relative pose between views C0 and C1.
    Pose3 c0Tc1 =
        wTc0.inverse(H1 ? &H_1_inverse : nullptr)
            .compose(wTc1, H1 ? &H_T_1 : nullptr, H2 ? &H_T_2 : nullptr);
    Rot3 R = c0Tc1.rotation(H1 || H2 ? &H_R_T : nullptr);
    Point3 t = c0Tc1.translation(H1 || H2 ? &H_t_T : nullptr);

    Matrix3 Kinv = K_.inverse();

    // Convert pixel coordinates to homogenous coordinates.
    Vector3 u0(u0_(0), u0_(1), 1), u1(u1_(0), u1_(1), 1);

    Vector3 f0 = Kinv * u0, f1 = Kinv * u1;

    Vector3 m0 = R.rotate(f0, H1 || H2 ? &H_m0_R : nullptr), m1 = f1;

    Unit3 m0_hat(m0), m1_hat(m1);

    std::vector<Vector3> m_primes;

    switch (minimizationType_) {
      case L1:
        m_primes = l1TriangulationError(
            t, m0, m1, m0_hat, m1_hat, H1 || H2 ? &H_t : nullptr,
            H1 || H2 ? &H_m0 : nullptr, H1 || H2 ? &H_m1 : nullptr);
        break;
      case L2:
        m_primes = l2TriangulationError(
            t, m0, m1, m0_hat, m1_hat, H1 || H2 ? &H_t : nullptr,
            H1 || H2 ? &H_m0 : nullptr, H1 || H2 ? &H_m1 : nullptr);
        break;
      case Linfinity:
        m_primes = lInfinityTriangulationError(
            t, m0, m1, m0_hat, m1_hat, H1 || H2 ? &H_t : nullptr,
            H1 || H2 ? &H_m0 : nullptr, H1 || H2 ? &H_m1 : nullptr);
        break;
    }

    Vector3 m0_prime = m_primes.at(0);
    Vector3 m1_prime = m_primes.at(1);

    // Angle between Rf0 and Rf0'
    Vector3 Rf0 = R.rotate(f0, H1 || H2 ? &H_Rf0_R : nullptr);
    double theta0 =
        vectorAngle(Rf0, m0_prime, H1 || H2 ? &H_theta0_Rf0 : nullptr,
                    H1 || H2 ? &H_theta0_m0_prime : nullptr);

    // Angle betwen f1 and f1'
    double theta1 = vectorAngle(f1, m1_prime, H1 || H2 ? &H_theta1_f1 : nullptr,
                                H1 || H2 ? &H_theta1_m1_prime : nullptr);

    if (H1) {
      Matrix3 H_m0_prime_m0 = H_m0.block<3, 3>(0, 0);
      // Matrix3 H_m0_prime_m1 = H_m1.block<3, 3>(0, 0);  // This is zero so we
      // don't include it.
      Matrix3 H_m1_prime_m0 = H_m0.block<3, 3>(3, 0);
      // Matrix3 H_m1_prime_m1 = H_m1.block<3, 3>(3, 0);

      Matrix3 H_m0_prime_t = H_t.block<3, 3>(0, 0);
      Matrix3 H_m1_prime_t = H_t.block<3, 3>(3, 0);

      Matrix16 J11 =
          (H_theta0_Rf0 * H_Rf0_R * H_R_T * H_T_1 * H_1_inverse) +  //
          (H_theta0_m0_prime * H_m0_prime_m0 * H_m0_R * H_R_T * H_T_1 *
           H_1_inverse) +  //
          //  (H_theta0_m0_prime * H_m0_prime_m1) // m1 is constant so 0
          (H_theta0_m0_prime * H_m0_prime_t * H_t_T * H_T_1 * H_1_inverse);

      Matrix16 J21 =
          // (H_theta1_f1) +  // This is zero because f1 is constant
          (H_theta1_m1_prime * H_m1_prime_m0 * H_m0_R * H_R_T * H_T_1 *
           H_1_inverse) +  //
          // (H_theta1_m1_prime * H_m1_prime_m1) // m1 is constant so 0
          (H_theta1_m1_prime * H_m1_prime_t * H_t_T * H_T_1 * H_1_inverse);

      (*H1) = Matrix26::Zero();
      H1->block<1, 6>(0, 0) = J11;
      H1->block<1, 6>(1, 0) = J21;
    }

    if (H2) {
      Matrix3 H_m0_prime_m0 = H_m0.block<3, 3>(0, 0);
      // Matrix3 H_m0_prime_m1 = H_m1.block<3, 3>(0, 0);  // This is zero so we
      // don't include it.
      Matrix3 H_m1_prime_m0 = H_m0.block<3, 3>(3, 0);
      // Matrix3 H_m1_prime_m1 = H_m1.block<3, 3>(3, 0);

      Matrix3 H_m0_prime_t = H_t.block<3, 3>(0, 0);
      Matrix3 H_m1_prime_t = H_t.block<3, 3>(3, 0);

      Matrix16 J12 =
          (H_theta0_Rf0 * H_Rf0_R * H_R_T * H_T_2) +                      //
          (H_theta0_m0_prime * H_m0_prime_m0 * H_m0_R * H_R_T * H_T_2) +  //
          // (H_theta0_m0_prime * H_m0_prime_m1) // m1 is constant so 0
          (H_theta0_m0_prime * H_m0_prime_t * H_t_T * H_T_2);

      Matrix16 J22 =
          // (H_theta1_f1) +  // f1 is constant
          (H_theta1_m1_prime * H_m1_prime_m0 * H_m0_R * H_R_T * H_T_2) +  //
          // (H_theta1_m1_prime * H_m1_prime_m1) // m1 is constant so 0
          (H_theta1_m1_prime * H_m1_prime_t * H_t_T * H_T_2);

      (*H2) = Matrix26::Zero();
      H2->block<1, 6>(0, 0) = J12;
      H2->block<1, 6>(1, 0) = J22;
    }
    // For L1 triangulation, we are trying to minimize θ₀+θ₁.
    Vector2 error(theta0, theta1);
    return error;
  }
};

}  // namespace gtsam
