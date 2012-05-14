/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3Q.cpp
 * @brief   Rotation (internal: quaternion representation*)
 * @author  Richard Roberts
 */

#ifdef GTSAM_DEFAULT_QUATERNIONS

#include <boost/math/constants/constants.hpp>
#include <gtsam/geometry/Rot3.h>

using namespace std;

namespace gtsam {

	static const Matrix I3 = eye(3);

  /* ************************************************************************* */
	Rot3::Rot3() : quaternion_(Quaternion::Identity()) {}

  /* ************************************************************************* */
	Rot3::Rot3(const Point3& r1, const Point3& r2, const Point3& r3) :
      quaternion_((Eigen::Matrix3d() <<
          r1.x(), r2.x(), r3.x(),
          r1.y(), r2.y(), r3.y(),
          r1.z(), r2.z(), r3.z()).finished()) {}

  /* ************************************************************************* */
  Rot3::Rot3(double R11, double R12, double R13,
      double R21, double R22, double R23,
      double R31, double R32, double R33) :
        quaternion_((Eigen::Matrix3d() <<
            R11, R12, R13,
            R21, R22, R23,
            R31, R32, R33).finished()) {}

  /* ************************************************************************* */
  Rot3::Rot3(const Matrix& R) :
      quaternion_(Eigen::Matrix3d(R)) {}

//  /* ************************************************************************* */
//   Rot3::Rot3(const Matrix3& R) :
//       quaternion_(R) {}

  /* ************************************************************************* */
  Rot3::Rot3(const Quaternion& q) : quaternion_(q) {}

  /* ************************************************************************* */
  Rot3 Rot3::Rx(double t) { return Quaternion(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitX())); }

  /* ************************************************************************* */
  Rot3 Rot3::Ry(double t) { return Quaternion(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitY())); }

  /* ************************************************************************* */
  Rot3 Rot3::Rz(double t) { return Quaternion(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ())); }

  /* ************************************************************************* */
  Rot3 Rot3::RzRyRx(double x, double y, double z) { return Rot3(
      Quaternion(Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())) *
      Quaternion(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())) *
      Quaternion(Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())));
  }

  /* ************************************************************************* */
  Rot3 Rot3::rodriguez(const Vector& w, double theta) {
    return Quaternion(Eigen::AngleAxisd(theta, w)); }

  /* ************************************************************************* */
  Rot3 Rot3::rodriguez(const Vector& w) {
    double t = w.norm();
    if (t < 1e-10) return Rot3();
    return rodriguez(w/t, t);
  }

  /* ************************************************************************* */
  bool Rot3::equals(const Rot3 & R, double tol) const {
    return equal_with_abs_tol(matrix(), R.matrix(), tol);
  }

  /* ************************************************************************* */
  Rot3 Rot3::compose(const Rot3& R2,
  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
    if (H1) *H1 = R2.transpose();
    if (H2) *H2 = I3;
    return Rot3(quaternion_ * R2.quaternion_);
  }

  /* ************************************************************************* */
  Point3 Rot3::operator*(const Point3& p) const {
    Eigen::Vector3d r = quaternion_ * Eigen::Vector3d(p.x(), p.y(), p.z());
    return Point3(r(0), r(1), r(2));
  }

  /* ************************************************************************* */
  Rot3 Rot3::inverse(boost::optional<Matrix&> H1) const {
    if (H1) *H1 = -matrix();
    return Rot3(quaternion_.inverse());
  }

  /* ************************************************************************* */
  Rot3 Rot3::between(const Rot3& R2,
  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
    if (H1) *H1 = -(R2.transpose()*matrix());
    if (H2) *H2 = I3;
    return between_default(*this, R2);
  }

  /* ************************************************************************* */
  Rot3 Rot3::operator*(const Rot3& R2) const {
    return Rot3(quaternion_ * R2.quaternion_);
  }

  /* ************************************************************************* */
  Point3 Rot3::rotate(const Point3& p,
        boost::optional<Matrix&> H1,  boost::optional<Matrix&> H2) const {
    Matrix R = matrix();
    if (H1) *H1 = R * skewSymmetric(-p.x(), -p.y(), -p.z());
    if (H2) *H2 = R;
    Eigen::Vector3d r = R * p.vector();
    return Point3(r.x(), r.y(), r.z());
  }

  /* ************************************************************************* */
  // see doc/math.lyx, SO(3) section
  Point3 Rot3::unrotate(const Point3& p,
      boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
    const Matrix Rt(transpose());
    Point3 q(Rt*p.vector()); // q = Rt*p
    if (H1) *H1 = skewSymmetric(q.x(), q.y(), q.z());
    if (H2) *H2 = Rt;
    return q;
  }

  /* ************************************************************************* */
  // Log map at identity - return the canonical coordinates of this rotation
  Vector3 Rot3::Logmap(const Rot3& R) {
    Eigen::AngleAxisd angleAxis(R.quaternion_);
    if(angleAxis.angle() > M_PI)      // Important:  use the smallest possible
      angleAxis.angle() -= 2.0*M_PI;  // angle, e.g. no more than PI, to keep
    if(angleAxis.angle() < -M_PI)     // error continuous.
      angleAxis.angle() += 2.0*M_PI;
    return angleAxis.axis() * angleAxis.angle();
  }

  /* ************************************************************************* */
	Rot3 Rot3::retract(const Vector& omega, Rot3::CoordinatesMode mode) const {
		return compose(Expmap(omega));
	}

	/* ************************************************************************* */
	Vector3 Rot3::localCoordinates(const Rot3& t2, Rot3::CoordinatesMode mode) const {
		return Logmap(between(t2));
	}

  /* ************************************************************************* */
  Matrix3 Rot3::matrix() const { return quaternion_.toRotationMatrix(); }

  /* ************************************************************************* */
  Matrix3 Rot3::transpose() const { return quaternion_.toRotationMatrix().transpose(); }

  /* ************************************************************************* */
  Point3 Rot3::column(int index) const{
    if(index == 3)
      return r3();
    else if(index == 2)
      return r2();
    else if(index == 1)
      return r1(); // default returns r1
    else
      throw invalid_argument("Argument to Rot3::column must be 1, 2, or 3");
  }

  /* ************************************************************************* */
  Point3 Rot3::r1() const { return Point3(quaternion_.toRotationMatrix().col(0)); }

  /* ************************************************************************* */
  Point3 Rot3::r2() const { return Point3(quaternion_.toRotationMatrix().col(1)); }

  /* ************************************************************************* */
  Point3 Rot3::r3() const { return Point3(quaternion_.toRotationMatrix().col(2)); }

  /* ************************************************************************* */
  Vector3 Rot3::xyz() const {
    Matrix I;Vector3 q;
    boost::tie(I,q)=RQ(matrix());
    return q;
  }

  /* ************************************************************************* */
  Vector3 Rot3::ypr() const {
  	Vector3 q = xyz();
    return Vector3(q(2),q(1),q(0));
  }

  /* ************************************************************************* */
  Vector3 Rot3::rpy() const {
  	Vector3 q = xyz();
    return Vector3(q(0),q(1),q(2));
  }

  /* ************************************************************************* */
  Quaternion Rot3::toQuaternion() const { return quaternion_; }

  /* ************************************************************************* */
  pair<Matrix3, Vector3> RQ(const Matrix3& A) {

    double x = -atan2(-A(2, 1), A(2, 2));
    Rot3 Qx = Rot3::Rx(-x);
    Matrix3 B = A * Qx.matrix();

    double y = -atan2(B(2, 0), B(2, 2));
    Rot3 Qy = Rot3::Ry(-y);
    Matrix3 C = B * Qy.matrix();

    double z = -atan2(-C(1, 0), C(1, 1));
    Rot3 Qz = Rot3::Rz(-z);
    Matrix3 R = C * Qz.matrix();

    Vector xyz = Vector3(x, y, z);
    return make_pair(R, xyz);
  }

} // namespace gtsam

#endif
