/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Similarity3.cpp
 * @brief  Implementation of Similarity3 transform
 * @author Paul Drews
 */

#include <gtsam_unstable/geometry/Similarity3.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

Similarity3::Similarity3() :
    R_(), t_(), s_(1) {
}

Similarity3::Similarity3(double s) :
    s_(s) {
}

Similarity3::Similarity3(const Rot3& R, const Point3& t, double s) :
    R_(R), t_(t), s_(s) {
}

Similarity3::Similarity3(const Matrix3& R, const Vector3& t, double s) :
    R_(R), t_(t), s_(s) {
}

Similarity3::Similarity3(const Matrix4& T) :
    R_(T.topLeftCorner<3, 3>()), t_(T.topRightCorner<3, 1>()), s_(1.0 / T(3, 3)) {
}

bool Similarity3::equals(const Similarity3& other, double tol) const {
  return R_.equals(other.R_, tol) && t_.equals(other.t_, tol)
      && s_ < (other.s_ + tol) && s_ > (other.s_ - tol);
}

bool Similarity3::operator==(const Similarity3& other) const {
  return R_.matrix() == other.R_.matrix() && t_ == other.t_ && s_ == other.s_;
}

void Similarity3::print(const std::string& s) const {
  std::cout << std::endl;
  std::cout << s;
  rotation().print("R:\n");
  translation().print("t: ");
  std::cout << "s: " << scale() << std::endl;
}

Similarity3 Similarity3::identity() {
  return Similarity3();
}
Similarity3 Similarity3::operator*(const Similarity3& T) const {
  return Similarity3(R_ * T.R_, ((1.0 / T.s_) * t_) + R_ * T.t_, s_ * T.s_);
}

Similarity3 Similarity3::inverse() const {
  Rot3 Rt = R_.inverse();
  Point3 sRt = R_.inverse() * (-s_ * t_);
  return Similarity3(Rt, sRt, 1.0 / s_);
}

Point3 Similarity3::transform_from(const Point3& p, //
    OptionalJacobian<3, 7> H1, OptionalJacobian<3, 3> H2) const {
  Point3 q = R_ * p + t_;
  if (H1) {
    const Matrix3 R = R_.matrix();
    Matrix3 DR = s_ * R * skewSymmetric(-p.x(), -p.y(), -p.z());
    // TODO(frank): explain the derivative in lambda
    *H1 << DR, s_ * R, s_ * p.vector();
  }
  if (H2)
    *H2 = s_ * R_.matrix(); // just 3*3 sub-block of matrix()
  return s_ * q;
}

Point3 Similarity3::operator*(const Point3& p) const {
  return transform_from(p);
}

Matrix4 Similarity3::wedge(const Vector7& xi) {
  // http://www.ethaneade.org/latex2html/lie/node29.html
  const auto w = xi.head<3>();
  const auto u = xi.segment<3>(3);
  double lambda = xi[6];
  Matrix4 W;
  W << skewSymmetric(w), u, 0, 0, 0, -lambda;
  return W;
}

Matrix7 Similarity3::AdjointMap() const {
  // http://www.ethaneade.org/latex2html/lie/node30.html
  const Matrix3 R = R_.matrix();
  const Vector3 t = t_.vector();
  const Matrix3 A = s_ * skewSymmetric(t) * R;
  Matrix7 adj;
  adj <<
      R, Z_3x3, Matrix31::Zero(), // 3*7
      A, s_ * R, -s_ * t, // 3*7
      Matrix16::Zero(), 1; // 1*7
  return adj;
}

Matrix3 Similarity3::GetV(Vector3 w, double lambda) {
  // http://www.ethaneade.org/latex2html/lie/node29.html
  double lambda2 = lambda * lambda;
  double theta2 = w.transpose() * w;
  double theta = sqrt(theta2);
  double A, B, C;
  // TODO(frank): eliminate copy/paste
  if (theta2 > 1e-9 && lambda2 > 1e-9) {
    const double X = sin(theta) / theta;
    const double Y = (1 - cos(theta)) / theta2;
    const double Z = (1 - X) / theta2;
    const double W = (0.5 - Y) / theta2;
    const double alpha = lambda2 / (lambda2 + theta2);
    const double beta = (exp(-lambda) - 1 + lambda) / lambda2;
    const double gamma = Y - (lambda * Z);
    const double mu = (1 - lambda + (0.5 * lambda2) - exp(-lambda))
        / (lambda2 * lambda);
    const double upsilon = Z - (lambda * W);
    A = (1 - exp(-lambda)) / lambda;
    B = alpha * (beta - gamma) + gamma;
    C = alpha * (mu - upsilon) + upsilon;
  } else if (theta2 <= 1e-9 && lambda2 > 1e-9) {
    //Taylor series expansions
    const double Y = 0.5 - theta2 / 24.0;
    const double Z = 1.0 / 6.0 - theta2 / 120.0;
    const double W = 1.0 / 24.0 - theta2 / 720.0;
    const double alpha = lambda2 / (lambda2 + theta2);
    const double beta = (exp(-lambda) - 1 + lambda) / lambda2;
    const double gamma = Y - (lambda * Z);
    const double mu = (1 - lambda + (0.5 * lambda2) - exp(-lambda))
        / (lambda2 * lambda);
    const double upsilon = Z - (lambda * W);
    A = (1 - exp(-lambda)) / lambda;
    B = alpha * (beta - gamma) + gamma;
    C = alpha * (mu - upsilon) + upsilon;
  } else if (theta2 > 1e-9 && lambda2 <= 1e-9) {
    const double X = sin(theta) / theta;
    const double Y = (1 - cos(theta)) / theta2;
    const double Z = (1 - X) / theta2;
    const double W = (0.5 - Y) / theta2;
    const double alpha = lambda2 / (lambda2 + theta2);
    const double beta = 0.5 - lambda / 6.0 + lambda2 / 24.0
        - (lambda * lambda2) / 120;
    const double gamma = Y - (lambda * Z);
    const double mu = 1.0 / 6.0 - lambda / 24 + lambda2 / 120
        - (lambda * lambda2) / 720;
    const double upsilon = Z - (lambda * W);
    if (lambda < 1e-9) {
      A = 1 - lambda / 2.0 + lambda2 / 6.0;
    } else {
      A = (1 - exp(-lambda)) / lambda;
    }
    B = alpha * (beta - gamma) + gamma;
    C = alpha * (mu - upsilon) + upsilon;
  } else {
    const double Y = 0.5 - theta2 / 24.0;
    const double Z = 1.0 / 6.0 - theta2 / 120.0;
    const double W = 1.0 / 24.0 - theta2 / 720.0;
    const double gamma = Y - (lambda * Z);
    const double upsilon = Z - (lambda * W);
    if (lambda < 1e-9) {
      A = 1 - lambda / 2.0 + lambda2 / 6.0;
    } else {
      A = (1 - exp(-lambda)) / lambda;
    }
    B = gamma;
    C = upsilon;
  }
  const Matrix3 Wx = skewSymmetric(w[0], w[1], w[2]);
  return A * I_3x3 + B * Wx + C * Wx * Wx;
}

Vector7 Similarity3::Logmap(const Similarity3& T, OptionalJacobian<7, 7> Hm) {
  // To get the logmap, calculate w and lambda, then solve for u as shown by Ethan at
  // www.ethaneade.org/latex2html/lie/node29.html
  const Vector3 w = Rot3::Logmap(T.R_);
  const double lambda = log(T.s_);
  Vector7 result;
  result << w, GetV(w, lambda).inverse() * T.t_.vector(), lambda;
  if (Hm) {
    throw std::runtime_error("Similarity3::Logmap: derivative not implemented");
  }
  return result;
}

Similarity3 Similarity3::Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm) {
  const auto w = v.head<3>();
  const auto u = v.segment<3>(3);
  const double lambda = v[6];
  if (Hm) {
    throw std::runtime_error("Similarity3::Expmap: derivative not implemented");
  }
  const Matrix3 V = GetV(w, lambda);
  return Similarity3(Rot3::Expmap(w), Point3(V * u), exp(lambda));
}

std::ostream &operator<<(std::ostream &os, const Similarity3& p) {
  os << "[" << p.rotation().xyz().transpose() << " "
      << p.translation().vector().transpose() << " " << p.scale() << "]\';";
  return os;
}

const Matrix4 Similarity3::matrix() const {
  Matrix4 T;
  T.topRows<3>() << R_.matrix(), t_.vector();
  T.bottomRows<1>() << 0, 0, 0, 1.0/s_;
  return T;
}

Similarity3::operator Pose3() const {
  return Pose3(R_, s_ * t_);
}

}
