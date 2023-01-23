/* ----------------------------------------------------------------------------

 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file Unit3.h
 * @date Feb 02, 2011
 * @author Can Erdogan
 * @author Frank Dellaert
 * @author Alex Trevor
 * @author Zhaoyang Lv
 * @brief The Unit3 class - basically a point on a unit sphere
 */

#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/config.h>  // for GTSAM_USE_TBB

#include <iostream>
#include <limits>
#include <cmath>
#include <vector>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Unit3::Unit3(const Vector3& p) : p_(p.normalized()) {}

Unit3::Unit3(double x, double y, double z) : p_(x, y, z) { p_.normalize(); }

Unit3::Unit3(const Point2& p, double f) : p_(p.x(), p.y(), f) {
  p_.normalize();
}
/* ************************************************************************* */
Unit3 Unit3::FromPoint3(const Point3& point, OptionalJacobian<2, 3> H) {
  // 3*3 Derivative of representation with respect to point is 3*3:
  Matrix3 D_p_point;
  Unit3 direction;
  direction.p_ = normalize(point, H ? &D_p_point : 0);
  if (H)
    *H << direction.basis().transpose() * D_p_point;
  return direction;
}

/* ************************************************************************* */
Unit3 Unit3::Random(std::mt19937& rng) {
  // http://mathworld.wolfram.com/SpherePointPicking.html
  // Adapted from implementation in boost, but using std <random>
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  double sqsum;
  double x, y;
  do {
    x = uniform(rng);
    y = uniform(rng);
    sqsum = x * x + y * y;
  } while (sqsum > 1);
  const double mult = 2 * sqrt(1 - sqsum);
  return Unit3(x * mult, y * mult, 2 * sqsum - 1);
}

/* ************************************************************************* */
// Get the axis of rotation with the minimum projected length of the point
static Point3 CalculateBestAxis(const Point3& n) {
  double mx = std::abs(n.x()), my = std::abs(n.y()), mz = std::abs(n.z());
  if ((mx <= my) && (mx <= mz)) {
    return Point3(1.0, 0.0, 0.0);
  } else if ((my <= mx) && (my <= mz)) {
    return Point3(0.0, 1.0, 0.0);
  } else {
    return Point3(0, 0, 1);
  }
}

/* ************************************************************************* */
const Matrix32& Unit3::basis(OptionalJacobian<6, 2> H) const {
#ifdef GTSAM_USE_TBB
  // NOTE(hayk): At some point it seemed like this reproducably resulted in
  // deadlock. However, I don't know why and I can no longer reproduce it.
  // It either was a red herring or there is still a latent bug left to debug.
  std::unique_lock<std::mutex> lock(B_mutex_);
#endif

  const bool cachedBasis = static_cast<bool>(B_);
  const bool cachedJacobian = static_cast<bool>(H_B_);

  if (H) {
    if (!cachedJacobian) {
      // Compute Jacobian. Recomputes B_
      Matrix32 B;
      Matrix62 jacobian;
      Matrix33 H_B1_n, H_b1_B1, H_b2_n, H_b2_b1;

      // Choose the direction of the first basis vector b1 in the tangent plane
      // by crossing n with the chosen axis.
      const Point3 n(p_), axis = CalculateBestAxis(n);
      const Point3 B1 = gtsam::cross(n, axis, &H_B1_n);

      // Normalize result to get a unit vector: b1 = B1 / |B1|.
      B.col(0) = normalize(B1, &H_b1_B1);

      // Get the second basis vector b2, which is orthogonal to n and b1.
      B.col(1) = gtsam::cross(n, B.col(0), &H_b2_n, &H_b2_b1);

      // Chain rule tomfoolery to compute the jacobian.
      const Matrix32& H_n_p = B;
      jacobian.block<3, 2>(0, 0) = H_b1_B1 * H_B1_n * H_n_p;
      auto H_b1_p = jacobian.block<3, 2>(0, 0);
      jacobian.block<3, 2>(3, 0) = H_b2_n * H_n_p + H_b2_b1 * H_b1_p;

      // Cache the result and jacobian
      H_B_ = (jacobian);
      B_ = (B);
    }

    // Return cached jacobian, possibly computed just above
    *H = *H_B_;
  }

  if (!cachedBasis) {
    // Same calculation as above, without derivatives.
    // Done after H block, as that possibly computes B_ for the first time
    Matrix32 B;

    const Point3 n(p_), axis = CalculateBestAxis(n);
    const Point3 B1 = gtsam::cross(n, axis);
    B.col(0) = normalize(B1);
    B.col(1) = gtsam::cross(n, B.col(0));
    B_ = (B);
  }

  return *B_;
}

/* ************************************************************************* */
Point3 Unit3::point3(OptionalJacobian<3, 2> H) const {
  if (H)
    *H = basis();
  return Point3(p_);
}

/* ************************************************************************* */
Vector3 Unit3::unitVector(OptionalJacobian<3, 2> H) const {
  if (H)
    *H = basis();
  return p_;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const Unit3& pair) {
  os << pair.p_ << endl;
  return os;
}

/* ************************************************************************* */
void Unit3::print(const std::string& s) const {
  cout << s << ":" << p_ << endl;
}

/* ************************************************************************* */
Matrix3 Unit3::skew() const {
  return skewSymmetric(p_.x(), p_.y(), p_.z());
}

/* ************************************************************************* */
double Unit3::dot(const Unit3& q, OptionalJacobian<1, 2> H_p,
                  OptionalJacobian<1, 2> H_q) const {
  // Get the unit vectors of each, and the derivative.
  Matrix32 H_pn_p;
  Point3 pn = point3(H_p ? &H_pn_p : nullptr);

  Matrix32 H_qn_q;
  const Point3 qn = q.point3(H_q ? &H_qn_q : nullptr);

  // Compute the dot product of the Point3s.
  Matrix13 H_dot_pn, H_dot_qn;
  double d = gtsam::dot(pn, qn, H_p ? &H_dot_pn : nullptr, H_q ? &H_dot_qn : nullptr);

  if (H_p) {
    (*H_p) << H_dot_pn * H_pn_p;
  }

  if (H_q) {
    (*H_q) = H_dot_qn * H_qn_q;
  }

  return d;
}

/* ************************************************************************* */
Vector2 Unit3::error(const Unit3& q, OptionalJacobian<2, 2> H_q) const {
  // 2D error is equal to B'*q, as B is 3x2 matrix and q is 3x1
  const Vector2 xi = basis().transpose() * q.p_;
  if (H_q) {
    *H_q = basis().transpose() * q.basis();
  }
  return xi;
}

/* ************************************************************************* */
Vector2 Unit3::errorVector(const Unit3& q, OptionalJacobian<2, 2> H_p,
                           OptionalJacobian<2, 2> H_q) const {
  // Get the point3 of this, and the derivative.
  Matrix32 H_qn_q;
  const Point3 qn = q.point3(H_q ? &H_qn_q : nullptr);

  // 2D error here is projecting q into the tangent plane of this (p).
  Matrix62 H_B_p;
  Matrix23 Bt = basis(H_p ? &H_B_p : nullptr).transpose();
  Vector2 xi = Bt * qn;

  if (H_p) {
    // Derivatives of each basis vector.
    const Matrix32& H_b1_p = H_B_p.block<3, 2>(0, 0);
    const Matrix32& H_b2_p = H_B_p.block<3, 2>(3, 0);

    // Derivatives of the two entries of xi wrt the basis vectors.
    const Matrix13 H_xi1_b1 = qn.transpose();
    const Matrix13 H_xi2_b2 = qn.transpose();

    // Assemble dxi/dp = dxi/dB * dB/dp.
    const Matrix12 H_xi1_p = H_xi1_b1 * H_b1_p;
    const Matrix12 H_xi2_p = H_xi2_b2 * H_b2_p;
    *H_p << H_xi1_p, H_xi2_p;
  }

  if (H_q) {
    // dxi/dq is given by dxi/dqu * dqu/dq, where qu is the unit vector of q.
    const Matrix23 H_xi_qu = Bt;
    *H_q = H_xi_qu * H_qn_q;
  }

  return xi;
}

/* ************************************************************************* */
double Unit3::distance(const Unit3& q, OptionalJacobian<1, 2> H) const {
  Matrix2 H_xi_q;
  const Vector2 xi = error(q, H ? &H_xi_q : nullptr);
  const double theta = xi.norm();
  if (H)
    *H = (xi.transpose() / theta) * H_xi_q;
  return theta;
}

/* ************************************************************************* */
Unit3 Unit3::retract(const Vector2& v, OptionalJacobian<2,2> H) const {
  // Compute the 3D xi_hat vector
  const Vector3 xi_hat = basis() * v;
  const double theta = xi_hat.norm();
  const double c = std::cos(theta);

  // Treat case of very small v differently.
  Matrix23 H_from_point;
  if (theta < std::numeric_limits<double>::epsilon()) {
    const Unit3 exp_p_xi_hat = Unit3::FromPoint3(c * p_ + xi_hat,
                                                 H? &H_from_point : nullptr);
    if (H) { // Jacobian
      *H = H_from_point *
          (-p_ * xi_hat.transpose() + Matrix33::Identity()) * basis();
    }
    return exp_p_xi_hat;
  }

  const double st = std::sin(theta) / theta;
  const Unit3 exp_p_xi_hat = Unit3::FromPoint3(c * p_ + xi_hat * st,
                                               H? &H_from_point : nullptr);
  if (H) { // Jacobian
    *H = H_from_point *
        (p_ * -st * xi_hat.transpose() + st * Matrix33::Identity() +
        xi_hat * ((c - st) / std::pow(theta, 2)) * xi_hat.transpose()) * basis();
  }
  return exp_p_xi_hat;
}

/* ************************************************************************* */
Vector2 Unit3::localCoordinates(const Unit3& other) const {
  const double x = p_.dot(other.p_);
  // Crucial quantity here is y = theta/sin(theta) with theta=acos(x)
  // Now, y = acos(x) / sin(acos(x)) = acos(x)/sqrt(1-x^2)
  // We treat the special case 1 and -1 below
  const double x2 = x * x;
  const double z = 1 - x2;
  double y;
  if (z < std::numeric_limits<double>::epsilon()) {
    if (x > 0)  // first order expansion at x=1
      y = 1.0 - (x - 1.0) / 3.0;
    else  // cop out
      return Vector2(M_PI, 0.0);
  } else {
    // no special case
    y = acos(x) / sqrt(z);
  }
  return basis().transpose() * y * (other.p_ - x * p_);
}
/* ************************************************************************* */

}  // namespace gtsam
