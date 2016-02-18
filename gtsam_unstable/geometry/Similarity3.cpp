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
    t_(0,0,0), s_(1) {
}

Similarity3::Similarity3(double s) :
    t_(0,0,0), s_(s) {
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
  return R_.equals(other.R_, tol) && traits<Point3>::Equals(t_, other.t_, tol)
      && s_ < (other.s_ + tol) && s_ > (other.s_ - tol);
}

bool Similarity3::operator==(const Similarity3& other) const {
  return R_.matrix() == other.R_.matrix() && t_ == other.t_ && s_ == other.s_;
}

void Similarity3::print(const std::string& s) const {
  std::cout << std::endl;
  std::cout << s;
  rotation().print("R:\n");
  std::cout << "t: " << translation().transpose() << "s: " << scale() << std::endl;
}

Similarity3 Similarity3::identity() {
  return Similarity3();
}
Similarity3 Similarity3::operator*(const Similarity3& T) const {
  return Similarity3(R_ * T.R_, ((1.0 / T.s_) * t_) + R_ * T.t_, s_ * T.s_);
}

Similarity3 Similarity3::inverse() const {
  const Rot3 Rt = R_.inverse();
  const Point3 sRt = Rt * (-s_ * t_);
  return Similarity3(Rt, sRt, 1.0 / s_);
}

Point3 Similarity3::transform_from(const Point3& p, //
    OptionalJacobian<3, 7> H1, OptionalJacobian<3, 3> H2) const {
  const Point3 q = R_ * p + t_;
  if (H1) {
    // For this derivative, see LieGroups.pdf
    const Matrix3 sR = s_ * R_.matrix();
    const Matrix3 DR = sR * skewSymmetric(-p.x(), -p.y(), -p.z());
    *H1 << DR, sR, sR * p;
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
  const double lambda = xi[6];
  Matrix4 W;
  W << skewSymmetric(w), u, 0, 0, 0, -lambda;
  return W;
}

Matrix7 Similarity3::AdjointMap() const {
  // http://www.ethaneade.org/latex2html/lie/node30.html
  const Matrix3 R = R_.matrix();
  const Vector3 t = t_;
  const Matrix3 A = s_ * skewSymmetric(t) * R;
  Matrix7 adj;
  adj << R, Z_3x3, Matrix31::Zero(), // 3*7
  A, s_ * R, -s_ * t, // 3*7
  Matrix16::Zero(), 1; // 1*7
  return adj;
}

Matrix3 Similarity3::GetV(Vector3 w, double lambda) {
  // http://www.ethaneade.org/latex2html/lie/node29.html
  const double theta2 = w.transpose() * w;
  double Y, Z, W;
  if (theta2 > 1e-9) {
    const double theta = sqrt(theta2);
    const double X = sin(theta) / theta;
    Y = (1 - cos(theta)) / theta2;
    Z = (1 - X) / theta2;
    W = (0.5 - Y) / theta2;
  } else {
    // Taylor series expansion for theta=0, X not needed (as is 1)
    Y = 0.5 - theta2 / 24.0;
    Z = 1.0 / 6.0 - theta2 / 120.0;
    W = 1.0 / 24.0 - theta2 / 720.0;
  }
  const double lambda2 = lambda * lambda, lambda3 = lambda2 * lambda;
  double A, alpha = 0.0, beta, mu;
  if (lambda2 > 1e-9) {
    A = (1.0 - exp(-lambda)) / lambda;
    alpha = 1.0 / (1.0 + theta2 / lambda2);
    beta = (exp(-lambda) - 1 + lambda) / lambda2;
    mu = (1 - lambda + (0.5 * lambda2) - exp(-lambda)) / lambda3;
  } else {
    A = 1.0 - lambda / 2.0 + lambda2 / 6.0;
    beta = 0.5 - lambda / 6.0 + lambda2 / 24.0 - lambda3 / 120.0;
    mu = 1.0 / 6.0 - lambda / 24.0 + lambda2 / 120.0 - lambda3 / 720.0;
  }
  const double gamma = Y - (lambda * Z), upsilon = Z - (lambda * W);
  const double B = alpha * (beta - gamma) + gamma;
  const double C = alpha * (mu - upsilon) + upsilon;
  const Matrix3 Wx = skewSymmetric(w[0], w[1], w[2]);
  return A * I_3x3 + B * Wx + C * Wx * Wx;
}

Vector7 Similarity3::Logmap(const Similarity3& T, OptionalJacobian<7, 7> Hm) {
  // To get the logmap, calculate w and lambda, then solve for u as shown by Ethan at
  // www.ethaneade.org/latex2html/lie/node29.html
  const Vector3 w = Rot3::Logmap(T.R_);
  const double lambda = log(T.s_);
  Vector7 result;
  result << w, GetV(w, lambda).inverse() * T.t_, lambda;
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
      << p.translation().transpose() << " " << p.scale() << "]\';";
  return os;
}

const Matrix4 Similarity3::matrix() const {
  Matrix4 T;
  T.topRows<3>() << R_.matrix(), t_;
  T.bottomRows<1>() << 0, 0, 0, 1.0 / s_;
  return T;
}

Similarity3::operator Pose3() const {
  return Pose3(R_, s_ * t_);
}

} // namespace gtsam
