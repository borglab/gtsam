#include <gtsam/geometry/SO3.h>
#include <gtsam_unstable/tfg_inekf/TFGGeometry.h>

#include <iostream>

namespace {

using gtsam::Matrix3;
using gtsam::Vector3;

inline Matrix3 skew(const Vector3& w) { return gtsam::skewSymmetric(w); }

// Left Jacobian of SO(3): J_l(phi) = I + ((1-cos t)/t^2) W + ((t-sin t)/t^3)
// W^2
Matrix3 leftJacobian(const Vector3& phi) {
  const double t2 = phi.squaredNorm();
  const Matrix3 W = skew(phi);
  // t2 < 1e-8 (t < 1e-4) avoids catastrophic cancellation in 1-cos(t)
  if (t2 < 1e-8) return Matrix3::Identity() + 0.5 * W;  // 1st-order limit
  const double t = std::sqrt(t2);
  const double a = (1.0 - std::cos(t)) / t2;
  const double b = (t - std::sin(t)) / (t2 * t);
  return Matrix3::Identity() + a * W + b * (W * W);
}

Matrix3 leftJacobianInverse(const Vector3& phi) {
  const double t2 = phi.squaredNorm();
  const Matrix3 W = skew(phi);
  if (t2 < 1e-8) return Matrix3::Identity() - 0.5 * W;  // 1st-order limit
  const double t = std::sqrt(t2);
  const double half = 0.5 * t;
  const double alpha = half * std::cos(half) / std::sin(half);
  return Matrix3::Identity() - 0.5 * W + ((1.0 - alpha) / t2) * (W * W);
}

}  // namespace

namespace tfg {

TwoFrameGroup::TwoFrameGroup()
    : R(Rot3::Identity()),
      v(Vector3::Zero()),
      p(Vector3::Zero()),
      gamma_omega(Vector3::Zero()),
      gamma_accel(Vector3::Zero()) {}

TwoFrameGroup::TwoFrameGroup(const Rot3& R, const Vector3& v, const Vector3& p,
                             const Vector3& gamma_omega,
                             const Vector3& gamma_accel)
    : R(R), v(v), p(p), gamma_omega(gamma_omega), gamma_accel(gamma_accel) {}

TwoFrameGroup TwoFrameGroup::Identity() { return TwoFrameGroup(); }

TwoFrameGroup TwoFrameGroup::FromState(const Rot3& R, const Vector3& v,
                                       const Vector3& p, const Vector3& b_omega,
                                       const Vector3& b_accel) {
  return TwoFrameGroup(R, v, p, -(R * b_omega), -(R * b_accel));
}

Vector3 TwoFrameGroup::bias_omega() const { return R.unrotate(-gamma_omega); }
Vector3 TwoFrameGroup::bias_accel() const { return R.unrotate(-gamma_accel); }

TwoFrameGroup TwoFrameGroup::operator*(const TwoFrameGroup& Y) const {
  return TwoFrameGroup(R * Y.R, v + R * Y.v, p + R * Y.p,
                       gamma_omega + R * Y.gamma_omega,
                       gamma_accel + R * Y.gamma_accel);
}

TwoFrameGroup TwoFrameGroup::inverse() const {
  const Rot3 Ri = R.inverse();
  return TwoFrameGroup(Ri, Ri * (-v), Ri * (-p), Ri * (-gamma_omega),
                       Ri * (-gamma_accel));
}

TwoFrameGroup TwoFrameGroup::Expmap(const TangentVector& xi) {
  const Vector3 theta = xi.segment<3>(0);
  const Matrix3 Jl = leftJacobian(theta);
  return TwoFrameGroup(Rot3::Expmap(theta), Jl * xi.segment<3>(3),
                       Jl * xi.segment<3>(6), Jl * xi.segment<3>(9),
                       Jl * xi.segment<3>(12));
}

TwoFrameGroup::TangentVector TwoFrameGroup::Logmap() const {
  const Vector3 theta = Rot3::Logmap(R);
  const Matrix3 Jli = leftJacobianInverse(theta);
  TangentVector xi;
  xi.segment<3>(0) = theta;
  xi.segment<3>(3) = Jli * v;
  xi.segment<3>(6) = Jli * p;
  xi.segment<3>(9) = Jli * gamma_omega;
  xi.segment<3>(12) = Jli * gamma_accel;
  return xi;
}

TwoFrameGroup::Jacobian TwoFrameGroup::AdjointMap() const {
  const Matrix3 A = R.matrix();
  Jacobian Ad = Jacobian::Zero();
  Ad.block<3, 3>(0, 0) = A;
  Ad.block<3, 3>(3, 3) = A;
  Ad.block<3, 3>(6, 6) = A;
  Ad.block<3, 3>(9, 9) = A;
  Ad.block<3, 3>(12, 12) = A;
  Ad.block<3, 3>(3, 0) = skew(v) * A;
  Ad.block<3, 3>(6, 0) = skew(p) * A;
  Ad.block<3, 3>(9, 0) = skew(gamma_omega) * A;
  Ad.block<3, 3>(12, 0) = skew(gamma_accel) * A;
  return Ad;
}

gtsam::Matrix5 TwoFrameGroup::C_matrix() const {
  gtsam::Matrix5 C = gtsam::Matrix5::Zero();
  C.block<3, 3>(0, 0) = R.matrix();
  C.block<3, 1>(0, 3) = v;
  C.block<3, 1>(0, 4) = p;
  C(3, 3) = 1.0;
  C(4, 4) = 1.0;
  return C;
}

TwoFrameGroup phi(const TwoFrameGroup& X, const TwoFrameGroup& xi) {
  return xi * X;
}

}  // namespace tfg

namespace gtsam {

using G = tfg::TwoFrameGroup;
using T = traits<G>;

bool T::Equals(const G& X, const G& Y, double tol) {
  return X.R.equals(Y.R, tol) && (X.v - Y.v).norm() < tol &&
         (X.p - Y.p).norm() < tol &&
         (X.gamma_omega - Y.gamma_omega).norm() < tol &&
         (X.gamma_accel - Y.gamma_accel).norm() < tol;
}

void T::Print(const G& X, const std::string& str) {
  std::cout << str;
  X.R.print("  R: ");
  std::cout << "  v:      " << X.v.transpose() << "\n"
            << "  p:      " << X.p.transpose() << "\n"
            << "  gamma_w: " << X.gamma_omega.transpose() << "\n"
            << "  gamma_a: " << X.gamma_accel.transpose() << "\n";
}

G T::Identity() { return G::Identity(); }

G T::Compose(const G& X, const G& Y, ChartJacobian Hx, ChartJacobian Hy) {
  if (Hx) *Hx = Y.inverse().AdjointMap();
  if (Hy) *Hy = Jacobian::Identity();
  return X * Y;
}

G T::Between(const G& X, const G& Y, ChartJacobian Hx, ChartJacobian Hy) {
  const G result = X.inverse() * Y;
  if (Hx) *Hx = -result.inverse().AdjointMap();
  if (Hy) *Hy = Jacobian::Identity();
  return result;
}

G T::Inverse(const G& X, ChartJacobian H) {
  if (H) *H = -X.AdjointMap();
  return X.inverse();
}

G T::Expmap(const TangentVector& xi, ChartJacobian H) {
  if (!H) return G::Expmap(xi);
  const Vector3 theta = xi.segment<3>(0);
  const gtsam::so3::DexpFunctor local(theta);
  const auto J = local.Jacobian();
  const Matrix3 Jr = J.right();
  const Matrix3 Rt = local.expmap().transpose();

  H->setZero();
  H->block<3, 3>(0, 0) = Jr;
  for (int o : {3, 6, 9, 12}) {
    Matrix3 Hi;
    J.applyLeft(xi.segment<3>(o), &Hi);
    H->block<3, 3>(o, o) = Jr;
    H->block<3, 3>(o, 0) = Rt * Hi;
  }
  return G::Expmap(xi);
}

T::TangentVector T::Logmap(const G& X, ChartJacobian H) {
  const TangentVector xi = X.Logmap();
  if (H) {
    Jacobian Hexp;
    Expmap(xi, Hexp);
    *H = Hexp.inverse();
  }
  return xi;
}

T::Jacobian T::AdjointMap(const G& X) { return X.AdjointMap(); }

G T::Retract(const G& X, const TangentVector& xi) { return X * G::Expmap(xi); }

T::TangentVector T::Local(const G& X, const G& Y) {
  return (X.inverse() * Y).Logmap();
}

}  // namespace gtsam
