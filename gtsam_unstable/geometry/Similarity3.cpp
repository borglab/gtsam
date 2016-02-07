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
  Matrix3 A = s_ * skewSymmetric(t) * R;
  Matrix7 adj;
  adj <<
      R, Z_3x3, Matrix31::Zero(), // 3*7
      A, s_ * R, -s_ * t, // 3*7
      Matrix16::Zero(), 1; // 1*7
  return adj;
}

Matrix3 Similarity3::GetV(Vector3 w, double lambda) {
  Matrix3 wx = skewSymmetric(w[0], w[1], w[2]);
  double lambdasquared = lambda * lambda;
  double thetasquared = w.transpose() * w;
  double theta = sqrt(thetasquared);
  double X, Y, Z, W, alpha, beta, gama, mu, upsilon, A, B, C;
  if (thetasquared > 1e-9 && lambdasquared > 1e-9) {
    X = sin(theta) / theta;
    Y = (1 - cos(theta)) / thetasquared;
    Z = (1 - X) / thetasquared;
    W = (0.5 - Y) / thetasquared;
    alpha = lambdasquared / (lambdasquared + thetasquared);
    beta = (exp(-lambda) - 1 + lambda) / lambdasquared;
    gama = Y - (lambda * Z);
    mu = (1 - lambda + (0.5 * lambdasquared) - exp(-lambda))
        / (lambdasquared * lambda);
    upsilon = Z - (lambda * W);
    A = (1 - exp(-lambda)) / lambda;
    B = alpha * (beta - gama) + gama;
    C = alpha * (mu - upsilon) + upsilon;
  } else if (thetasquared <= 1e-9 && lambdasquared > 1e-9) {
    //Taylor series expansions
    X = 1;
    Y = 0.5 - thetasquared / 24.0;
    Z = 1.0 / 6.0 - thetasquared / 120.0;
    W = 1.0 / 24.0 - thetasquared / 720.0;
    alpha = lambdasquared / (lambdasquared + thetasquared);
    beta = (exp(-lambda) - 1 + lambda) / lambdasquared;
    gama = Y - (lambda * Z);
    mu = (1 - lambda + (0.5 * lambdasquared) - exp(-lambda))
        / (lambdasquared * lambda);
    upsilon = Z - (lambda * W);
    A = (1 - exp(-lambda)) / lambda;
    B = alpha * (beta - gama) + gama;
    C = alpha * (mu - upsilon) + upsilon;
  } else if (thetasquared > 1e-9 && lambdasquared <= 1e-9) {
    X = sin(theta) / theta;
    Y = (1 - cos(theta)) / thetasquared;
    Z = (1 - X) / thetasquared;
    W = (0.5 - Y) / thetasquared;
    alpha = lambdasquared / (lambdasquared + thetasquared);
    beta = 0.5 - lambda / 6.0 + lambdasquared / 24.0
        - (lambda * lambdasquared) / 120;
    gama = Y - (lambda * Z);
    mu = 1.0 / 6.0 - lambda / 24 + lambdasquared / 120
        - (lambda * lambdasquared) / 720;
    upsilon = Z - (lambda * W);
    if (lambda < 1e-9) {
      A = 1 - lambda / 2.0 + lambdasquared / 6.0;
    } else {
      A = (1 - exp(-lambda)) / lambda;
    }
    B = alpha * (beta - gama) + gama;
    C = alpha * (mu - upsilon) + upsilon;
  } else {
    X = 1;
    Y = 0.5 - thetasquared / 24.0;
    Z = 1.0 / 6.0 - thetasquared / 120.0;
    W = 1.0 / 24.0 - thetasquared / 720.0;
    alpha = lambdasquared / (lambdasquared + thetasquared);
    beta = 0.5 - lambda / 6.0 + lambdasquared / 24.0
        - (lambda * lambdasquared) / 120;
    gama = Y - (lambda * Z);
    mu = 1.0 / 6.0 - lambda / 24 + lambdasquared / 120
        - (lambda * lambdasquared) / 720;
    upsilon = Z - (lambda * W);
    if (lambda < 1e-9) {
      A = 1 - lambda / 2.0 + lambdasquared / 6.0;
    } else {
      A = (1 - exp(-lambda)) / lambda;
    }
    B = gama;
    C = upsilon;
  }
  return A * I_3x3 + B * wx + C * wx * wx;
}

Vector7 Similarity3::Logmap(const Similarity3& s, OptionalJacobian<7, 7> Hm) {
  // To get the logmap, calculate w and lambda, then solve for u as show at ethaneade.org
  // www.ethaneade.org/latex2html/lie/node29.html
  Vector3 w = Rot3::Logmap(s.R_);
  double lambda = log(s.s_);
  Vector7 result;
  result << w, GetV(w, lambda).inverse() * s.t_.vector(), lambda;
  if (Hm) {
    // incomplete
  }
  return result;
}

Similarity3 Similarity3::Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm) {
  const auto w = v.head<3>();
  const auto u = v.segment<3>(3);
  double lambda = v[6];
  if (Hm) {
    // Matrix6 J_pose = Pose3::ExpmapDerivative(v.head<6>());
    // incomplete
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
