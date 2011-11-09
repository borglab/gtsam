/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3Q.cpp
 * @brief   Rotation (internal: 3*3 matrix representation*)
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

#include <boost/math/constants/constants.hpp>
#include <gtsam/geometry/Rot3Q.h>
#include <gtsam/base/Lie-inl.h>

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Rot3Q);

	static const Matrix I3 = eye(3);

  /* ************************************************************************* */
	// static member functions to construct rotations

  // Considerably faster than composing matrices above !
  Rot3Q Rot3Q::RzRyRx(double x, double y, double z) { return Rot3Q(
      Quaternion(Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())) *
      Quaternion(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())) *
      Quaternion(Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())));
  }

  /* ************************************************************************* */
  Rot3Q Rot3Q::rodriguez(const Vector& w, double theta) {
    return Quaternion(Eigen::AngleAxisd(theta, w)); }

  /* ************************************************************************* */
  Rot3Q Rot3Q::rodriguez(const Vector& w) {
    double t = w.norm();
    if (t < 1e-10) return Rot3Q();
    return rodriguez(w/t, t);
  }

  /* ************************************************************************* */
  bool Rot3Q::equals(const Rot3Q & R, double tol) const {
    return equal_with_abs_tol(matrix(), R.matrix(), tol);
  }

  /* ************************************************************************* */
  Matrix Rot3Q::matrix() const { return quaternion_.toRotationMatrix(); }

  /* ************************************************************************* */
  Matrix Rot3Q::transpose() const { return quaternion_.toRotationMatrix().transpose(); }

  /* ************************************************************************* */
  Vector Rot3Q::xyz() const {
    Matrix I;Vector q;
    boost::tie(I,q)=RQ(matrix());
    return q;
  }

  Vector Rot3Q::ypr() const {
  	Vector q = xyz();
    return Vector_(3,q(2),q(1),q(0));
  }

  Vector Rot3Q::rpy() const {
  	Vector q = xyz();
    return Vector_(3,q(0),q(1),q(2));
  }

  /* ************************************************************************* */
  // Log map at identity - return the canonical coordinates of this rotation
  Vector Rot3Q::Logmap(const Rot3Q& R) {
    Eigen::AngleAxisd angleAxis(R.quaternion_);
    if(angleAxis.angle() > M_PI)      // Important:  use the smallest possible
      angleAxis.angle() -= 2.0*M_PI;  // angle, e.g. no more than PI, to keep
    if(angleAxis.angle() < -M_PI)     // error continuous.
      angleAxis.angle() += 2.0*M_PI;
    return angleAxis.axis() * angleAxis.angle();
  }

  /* ************************************************************************* */
  Point3 Rot3Q::rotate(const Point3& p,
  		  boost::optional<Matrix&> H1,  boost::optional<Matrix&> H2) const {
    Matrix R = matrix();
	  if (H1) *H1 = R * skewSymmetric(-p.x(), -p.y(), -p.z());
	  if (H2) *H2 = R;
	  Eigen::Vector3d r = R * p.vector();
	  return Point3(r.x(), r.y(), r.z());
  }

  /* ************************************************************************* */
  // see doc/math.lyx, SO(3) section
  Point3 Rot3Q::unrotate(const Point3& p,
  		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
    const Matrix Rt(transpose());
    Point3 q(Rt*p.vector()); // q = Rt*p
    if (H1) *H1 = skewSymmetric(q.x(), q.y(), q.z());
    if (H2) *H2 = Rt;
    return q;
  }

  /* ************************************************************************* */
  Rot3Q Rot3Q::compose(const Rot3Q& R2,
	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
		if (H1) *H1 = R2.transpose();
	  if (H2) *H2 = I3;
	  return Rot3Q(quaternion_ * R2.quaternion_);
  }

  /* ************************************************************************* */
  Rot3Q Rot3Q::between(const Rot3Q& R2,
	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) *H1 = -(R2.transpose()*matrix());
	  if (H2) *H2 = I3;
	  return between_default(*this, R2);
  }

  /* ************************************************************************* */

} // namespace gtsam
