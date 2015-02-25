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
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Manifold.h>
#include <gtsam_unstable/geometry/Similarity3.h>

namespace gtsam {

Similarity3::Similarity3(const Matrix3& R, const Vector3& t, double s) {
  R_ = R;
  t_ = t;
  s_ = s;
}

/// Return the translation
const Vector3 Similarity3::t() const {
  return t_.vector();
}

/// Return the rotation matrix
const Matrix3 Similarity3::R() const {
  return R_.matrix();
}

Similarity3::Similarity3() :
        R_(), t_(), s_(1){
}

/// Construct pure scaling
Similarity3::Similarity3(double s) {
  s_ = s;
}

/// Construct from GTSAM types
Similarity3::Similarity3(const Rot3& R, const Point3& t, double s) {
  R_ = R;
  t_ = t;
  s_ = s;
}

Similarity3::operator Pose3() const {
  return Pose3(R_, s_*t_);
}

Similarity3 Similarity3::identity() {
  //std::cout << "Identity!" << std::endl;
  return Similarity3(); }

Vector7 Similarity3::Logmap(const Similarity3& s, OptionalJacobian<7, 7> Hm) {
  std::cout << "Logmap!" << std::endl;
  return Vector7();
}

Similarity3 Similarity3::Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm) {
  std::cout << "Expmap!" << std::endl;
  return Similarity3();
}

bool Similarity3::operator==(const Similarity3& other) const {
  return (R_.equals(other.R_)) && (t_ == other.t_) && (s_ == other.s_);
}

/// Compare with tolerance
bool Similarity3::equals(const Similarity3& sim, double tol) const {
  return rotation().equals(sim.rotation(), tol) && translation().equals(sim.translation(), tol)
      && scale() < (sim.scale()+tol) && scale() > (sim.scale()-tol);
}

Point3 Similarity3::transform_from(const Point3& p) const {
  return R_ * (s_ * p) + t_;
}

Matrix7 Similarity3::AdjointMap() const{
  const Matrix3 R = R_.matrix();
  const Vector3 t = t_.vector();
  Matrix3 A = s_ * skewSymmetric(t) * R;
  Matrix7 adj;
  adj << s_*R, A, -s_*t, Z_3x3, R, Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 1, 6>::Zero(), 1;
  return adj;
}

/** syntactic sugar for transform_from */
inline Point3 Similarity3::operator*(const Point3& p) const {
  return transform_from(p);
}

Similarity3 Similarity3::inverse() const {
  Rot3 Rt = R_.inverse();
  Point3 sRt = R_.inverse() * (-s_ * t_);
  return Similarity3(Rt, sRt, 1.0/s_);
}

Similarity3 Similarity3::operator*(const Similarity3& T) const {
  return Similarity3(R_ * T.R_, ((1.0/T.s_)*t_) + R_ * T.t_, s_*T.s_);
}

void Similarity3::print(const std::string& s) const {
  std::cout << std::endl;
  std::cout << s;
  rotation().print("R:\n");
  translation().print("t: ");
  std::cout << "s: " << scale() << std::endl;
}

/// Return the rotation matrix
Rot3 Similarity3::rotation() const {
  return R_;
}

/// Return the translation
Point3 Similarity3::translation() const {
  return t_;
}

/// Return the scale
double Similarity3::scale() const {
  return s_;
}

/// Update Similarity transform via 7-dim vector in tangent space
Similarity3 Similarity3::ChartAtOrigin::Retract(const Vector7& v,  ChartJacobian H) {
  // Will retracting or localCoordinating R work if R is not a unit rotation?
  // Also, how do we actually get s out?  Seems like we need to store it somewhere.
  Rot3 r;  //Create a zero rotation to do our retraction.
  return Similarity3( //
      r.retract(v.head<3>()), // retract rotation using v[0,1,2]
      Point3(v.segment<3>(3)), // Retract the translation
      1.0 + v[6]); //finally, update scale using v[6]
}

/// 7-dimensional vector v in tangent space that makes other = this->retract(v)
Vector7 Similarity3::ChartAtOrigin::Local(const Similarity3& other,  ChartJacobian H) {
  Rot3 r;  //Create a zero rotation to do the retraction
  Vector7 v;
  v.head<3>() = r.localCoordinates(other.R_);
  v.segment<3>(3) = other.t_.vector();
  //v.segment<3>(3) = translation().localCoordinates(other.translation());
  v[6] = other.s_ - 1.0;
  return v;
}
}


