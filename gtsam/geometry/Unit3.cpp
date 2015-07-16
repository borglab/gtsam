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

#ifdef GTSAM_USE_TBB
#include <tbb/mutex.h>
#endif

#include <boost/random/variate_generator.hpp>
#include <iostream>
#include <limits>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Unit3 Unit3::FromPoint3(const Point3& point, OptionalJacobian<2,3> H) {
  Unit3 direction(point);
  if (H) {
    // 3*3 Derivative of representation with respect to point is 3*3:
    Matrix3 D_p_point;
    point.normalize(D_p_point); // TODO, this calculates norm a second time :-(
    // Calculate the 2*3 Jacobian
    *H << direction.basis().transpose() * D_p_point;
  }
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
  vector<double> d = generator();
  return Unit3(d[0], d[1], d[2]);
}

#ifdef GTSAM_USE_TBB
tbb::mutex unit3BasisMutex;
#endif

/* ************************************************************************* */
const Matrix32& Unit3::basis() const {
#ifdef GTSAM_USE_TBB
  tbb::mutex::scoped_lock lock(unit3BasisMutex);
#endif

  // Return cached version if exists
  if (B_) return *B_;

  // Get the axis of rotation with the minimum projected length of the point
  Vector3 axis;
  double mx = fabs(p_.x()), my = fabs(p_.y()), mz = fabs(p_.z());
  if ((mx <= my) && (mx <= mz))
    axis = Vector3(1.0, 0.0, 0.0);
  else if ((my <= mx) && (my <= mz))
    axis = Vector3(0.0, 1.0, 0.0);
  else if ((mz <= mx) && (mz <= my))
    axis = Vector3(0.0, 0.0, 1.0);
  else
    assert(false);

  // Create the two basis vectors
  Vector3 b1 = p_.cross(axis).normalized();
  Vector3 b2 = p_.cross(b1).normalized();

  // Create the basis matrix
  B_.reset(Matrix32());
  (*B_) << b1, b2;
  return *B_;
}

/* ************************************************************************* */
/// The print fuction
void Unit3::print(const std::string& s) const {
  cout << s << ":" << p_ << endl;
}

/* ************************************************************************* */
Matrix3 Unit3::skew() const {
  return skewSymmetric(p_.x(), p_.y(), p_.z());
}

/* ************************************************************************* */
Vector2 Unit3::error(const Unit3& q, OptionalJacobian<2,2> H) const {
  // 2D error is equal to B'*q, as B is 3x2 matrix and q is 3x1
  Vector2 xi = basis().transpose() * q.p_;
  if (H)
    *H = basis().transpose() * q.basis();
  return xi;
}

/* ************************************************************************* */
double Unit3::distance(const Unit3& q, OptionalJacobian<1,2> H) const {
  Matrix2 H_;
  Vector2 xi = error(q, H_);
  double theta = xi.norm();
  if (H)
    *H = (xi.transpose() / theta) * H_;
  return theta;
}

/* ************************************************************************* */
Unit3 Unit3::retract(const Vector2& v) const {
  // Compute the 3D xi_hat vector
  Vector3 xi_hat = basis() * v;
  double theta = xi_hat.norm();

  // Treat case of very small v differently
  if (theta < std::numeric_limits<double>::epsilon()) {
    return Unit3(cos(theta) * p_ + xi_hat);
  }

  Vector3 exp_p_xi_hat =
      cos(theta) * p_ + xi_hat * (sin(theta) / theta);
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
