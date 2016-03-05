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
#include <gtsam/config.h> // for GTSAM_USE_TBB

#ifdef __clang__
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-variable"
#endif
#include <boost/random/uniform_on_sphere.hpp>
#ifdef __clang__
#  pragma clang diagnostic pop
#endif

#include <boost/random/variate_generator.hpp>
#include <iostream>
#include <limits>
#include <cmath>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Unit3 Unit3::FromPoint3(const Point3& point, OptionalJacobian<2,3> H) {
  // 3*3 Derivative of representation with respect to point is 3*3:
  Matrix3 D_p_point;
  Unit3 direction;
  direction.p_ = normalize(point, H ? &D_p_point : 0);
  if (H)
    *H << direction.basis().transpose() * D_p_point;
  return direction;
}

/* ************************************************************************* */
Unit3 Unit3::Random(boost::mt19937 & rng) {
  // TODO allow any engine without including all of boost :-(
  boost::uniform_on_sphere<double> randomDirection(3);
  // This variate_generator object is required for versions of boost somewhere
  // around 1.46, instead of drawing directly using boost::uniform_on_sphere(rng).
  boost::variate_generator<boost::mt19937&, boost::uniform_on_sphere<double> > generator(
      rng, randomDirection);
  const vector<double> d = generator();
  return Unit3(d[0], d[1], d[2]);
}

/* ************************************************************************* */
const Matrix32& Unit3::basis(OptionalJacobian<6, 2> H) const {
#ifdef GTSAM_USE_TBB
  // NOTE(hayk): At some point it seemed like this reproducably resulted in deadlock. However, I
  // can't see the reason why and I can no longer reproduce it. It may have been a red herring, or
  // there is still a latent bug to watch out for.
  tbb::mutex::scoped_lock lock(B_mutex_);
#endif

  // Return cached basis if available and the Jacobian isn't needed.
  if (B_ && !H) {
    return *B_;
  }

  // Return cached basis and derivatives if available.
  if (B_ && H && H_B_) {
    *H = *H_B_;
    return *B_;
  }

  // Get the unit vector and derivative wrt this.
  // NOTE(hayk): We can't call point3(), because it would recursively call basis().
  const Point3 n(p_);

  // Get the axis of rotation with the minimum projected length of the point
  Point3 axis(0, 0, 1);
  double mx = fabs(n.x()), my = fabs(n.y()), mz = fabs(n.z());
  if ((mx <= my) && (mx <= mz)) {
    axis = Point3(1.0, 0.0, 0.0);
  } else if ((my <= mx) && (my <= mz)) {
    axis = Point3(0.0, 1.0, 0.0);
  }

  // Choose the direction of the first basis vector b1 in the tangent plane by crossing n with
  // the chosen axis.
  Matrix33 H_B1_n;
  Point3 B1 = gtsam::cross(n, axis, H ? &H_B1_n : nullptr);

  // Normalize result to get a unit vector: b1 = B1 / |B1|.
  Matrix33 H_b1_B1;
  Point3 b1 = normalize(B1, H ? &H_b1_B1 : nullptr);

  // Get the second basis vector b2, which is orthogonal to n and b1, by crossing them.
  // No need to normalize this, p and b1 are orthogonal unit vectors.
  Matrix33 H_b2_n, H_b2_b1;
  Point3 b2 = gtsam::cross(n, b1, H ? &H_b2_n : nullptr, H ? &H_b2_b1 : nullptr);

  // Create the basis by stacking b1 and b2.
  B_.reset(Matrix32());
  (*B_) << b1.x(), b2.x(), b1.y(), b2.y(), b1.z(), b2.z();

  if (H) {
    // Chain rule tomfoolery to compute the derivative.
    const Matrix32& H_n_p = *B_;
    const Matrix32 H_b1_p = H_b1_B1 * H_B1_n * H_n_p;
    const Matrix32 H_b2_p = H_b2_n * H_n_p + H_b2_b1 * H_b1_p;

    // Cache the derivative and fill the result.
    H_B_.reset(Matrix62());
    (*H_B_) << H_b1_p, H_b2_p;
    *H = *H_B_;
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
double Unit3::dot(const Unit3& q, OptionalJacobian<1,2> H_p, OptionalJacobian<1,2> H_q) const {
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
Vector2 Unit3::error(const Unit3& q, OptionalJacobian<2,2> H_q) const {
  // 2D error is equal to B'*q, as B is 3x2 matrix and q is 3x1
  const Vector2 xi = basis().transpose() * q.p_;
  if (H_q) {
    *H_q = basis().transpose() * q.basis();
  }
  return xi;
}

/* ************************************************************************* */
Vector2 Unit3::errorVector(const Unit3& q, OptionalJacobian<2, 2> H_p, OptionalJacobian<2, 2> H_q) const {
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
double Unit3::distance(const Unit3& q, OptionalJacobian<1,2> H) const {
  Matrix2 H_xi_q;
  const Vector2 xi = error(q, H ? &H_xi_q : nullptr);
  const double theta = xi.norm();
  if (H)
    *H = (xi.transpose() / theta) * H_xi_q;
  return theta;
}

/* ************************************************************************* */
Unit3 Unit3::retract(const Vector2& v) const {
  // Compute the 3D xi_hat vector
  const Vector3 xi_hat = basis() * v;
  const double theta = xi_hat.norm();

  // Treat case of very small v differently
  if (theta < std::numeric_limits<double>::epsilon()) {
    return Unit3(Vector3(std::cos(theta) * p_ + xi_hat));
  }

  const Vector3 exp_p_xi_hat =
      std::cos(theta) * p_ + xi_hat * (sin(theta) / theta);
  return Unit3(exp_p_xi_hat);
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

}
