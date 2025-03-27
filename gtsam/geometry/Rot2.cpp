/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2.cpp
 * @date Dec 9, 2009
 * @author Frank Dellaert
 * @brief 2D Rotations
 */

#include <gtsam/geometry/Rot2.h>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Rot2 Rot2::fromCosSin(double c, double s) {
  Rot2 R(c, s);
  return R.normalize();
}

/* ************************************************************************* */
Rot2 Rot2::atan2(double y, double x) {
  Rot2 R(x, y);
  return R.normalize();
}

/* ************************************************************************* */
Rot2 Rot2::Random(std::mt19937& rng) {
  uniform_real_distribution<double> randomAngle(-M_PI, M_PI);
  double angle = randomAngle(rng);
  return fromAngle(angle);
}

/* ************************************************************************* */
void Rot2::print(const string& s) const {
  cout << s << ": " << theta() << endl;
}

/* ************************************************************************* */
bool Rot2::equals(const Rot2& R, double tol) const {
  return std::abs(c_ - R.c_) <= tol && std::abs(s_ - R.s_) <= tol;
}

/* ************************************************************************* */
Rot2& Rot2::normalize() {
  double scale = c_*c_ + s_*s_;
  if(std::abs(scale-1.0) > 1e-10) {
    scale = 1 / sqrt(scale);
    c_ *= scale;
    s_ *= scale;
  }
  return *this;
}

/* ************************************************************************* */
Rot2 Rot2::Expmap(const Vector1& v, OptionalJacobian<1, 1> H) {
  if (H)
    *H = I_1x1;
  if (v.isZero())
    return (Rot2());
  else
    return Rot2::fromAngle(v(0));
}

/* ************************************************************************* */
Vector1 Rot2::Logmap(const Rot2& r, OptionalJacobian<1, 1> H) {
  if (H)
    *H = I_1x1;
  Vector1 v;
  v << r.theta();
  return v;
}
/* ************************************************************************* */
Matrix2 Rot2::Hat(const Vector1& xi) {
  Matrix2 X;
  X << 0., -xi.x(),
    xi.x(), 0.;
  return X;
}

/* ************************************************************************* */
Vector1 Rot2::Vee(const Matrix2& X) {
  TangentVector v;
  v << X(1, 0);
  return v;
}

/* ************************************************************************* */
Matrix2 Rot2::matrix() const {
  Matrix2 rvalue_;
  rvalue_ <<  c_, -s_, s_, c_;
  return rvalue_;
}

/* ************************************************************************* */
Matrix2 Rot2::transpose() const {
  Matrix2 rvalue_;
  rvalue_ <<   c_, s_, -s_, c_;
  return rvalue_;
}

/* ************************************************************************* */
// see doc/math.lyx, SO(2) section
Point2 Rot2::rotate(const Point2& p, OptionalJacobian<2, 1> H1,
    OptionalJacobian<2, 2> H2) const {
  const Point2 q = Point2(c_ * p.x() + -s_ * p.y(), s_ * p.x() + c_ * p.y());
  if (H1) *H1 << -q.y(), q.x();
  if (H2) *H2 = matrix();
  return q;
}

/* ************************************************************************* */
// see doc/math.lyx, SO(2) section
Point2 Rot2::unrotate(const Point2& p,
    OptionalJacobian<2, 1> H1, OptionalJacobian<2, 2> H2) const {
  const Point2 q = Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
  if (H1) *H1 << q.y(), -q.x();
  if (H2) *H2 = transpose();
  return q;
}

/* ************************************************************************* */
Rot2 Rot2::relativeBearing(const Point2& d, OptionalJacobian<1, 2> H) {
  double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
  if(std::abs(n) > 1e-5) {
    if (H) {
      *H << -y / d2, x / d2;
    }
    return Rot2::fromCosSin(x / n, y / n);
  } else {
    if (H) *H << 0.0, 0.0;
    return Rot2();
  }
}

/* ************************************************************************* */
Rot2 Rot2::ClosestTo(const Matrix2& M) {
  Eigen::JacobiSVD<Matrix2> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Matrix2& U = svd.matrixU();
  const Matrix2& V = svd.matrixV();
  const double det = (U * V.transpose()).determinant();
  Matrix2 M_prime = (U * Vector2(1, det).asDiagonal() * V.transpose());

  double c = M_prime(0, 0);
  double s = M_prime(1, 0);
  return Rot2::fromCosSin(c, s);
}

/* ************************************************************************* */
Vector4 Rot2::vec(OptionalJacobian<4, 1> H) const {
  // Vectorize
  const Matrix2 M = matrix();
  const Vector4 X = Eigen::Map<const Vector4>(M.data());

  // If requested, calculate H as (I_3 \oplus M) * G.
  if (H) {
    static const Matrix41 G = (Matrix41() << 0, 1, -1, 0).finished();
    for (size_t i = 0; i < 2; i++)
      H->block(i * 2, 0, 2, dimension) = M * G.block(i * 2, 0, 2, dimension);
  }

  return X;
}

/* ************************************************************************* */

} // gtsam
