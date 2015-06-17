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

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/base/Manifold.h>

namespace gtsam {

Similarity3::Similarity3(const Matrix3& R, const Vector3& t, double s) :
      R_(R), t_(t), s_(s) {
}

Similarity3::Similarity3() :
            R_(), t_(), s_(1){
}

Similarity3::Similarity3(double s) :
  s_ (s) {
}

Similarity3::Similarity3(const Rotation& R, const Translation& t, double s) :
      R_ (R), t_ (t), s_ (s) {
}

Similarity3::operator Pose3() const {
  return Pose3(R_, s_*t_);
}

Similarity3 Similarity3::identity() {
  return Similarity3(); }

//Vector7 Similarity3::Logmap(const Similarity3& s, OptionalJacobian<7, 7> Hm) {
//  return Vector7();
//}
//
//Similarity3 Similarity3::Expmap(const Vector7& v, OptionalJacobian<7, 7> Hm) {
//  return Similarity3();
//}

bool Similarity3::operator==(const Similarity3& other) const {
  return (R_.equals(other.R_)) && (t_ == other.t_) && (s_ == other.s_);
}

bool Similarity3::equals(const Similarity3& sim, double tol) const {
  return R_.equals(sim.R_, tol) && t_.equals(sim.t_, tol)
      && s_ < (sim.s_+tol) && s_ > (sim.s_-tol);
}

Similarity3::Translation Similarity3::transform_from(const Translation& p) const {
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

inline Similarity3::Translation Similarity3::operator*(const Translation& p) const {
  return transform_from(p);
}

Similarity3 Similarity3::inverse() const {
  Rotation Rt = R_.inverse();
  Translation sRt = R_.inverse() * (-s_ * t_);
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

std::ostream &operator<<(std::ostream &os, const Similarity3& p) {
  os << "[" << p.rotation().xyz().transpose() << " " << p.translation().vector().transpose() << " " <<
      p.scale() << "]\';";
  return os;
}

Similarity3 Similarity3::ChartAtOrigin::Retract(const Vector7& v,  ChartJacobian H) {
  // Will retracting or localCoordinating R work if R is not a unit rotation?
  // Also, how do we actually get s out?  Seems like we need to store it somewhere.
  Rotation r;  //Create a zero rotation to do our retraction.
  return Similarity3( //
      r.retract(v.head<3>()), // retract rotation using v[0,1,2]
      Translation(v.segment<3>(3)), // Retract the translation
      1.0 + v[6]); //finally, update scale using v[6]
}

Vector7 Similarity3::ChartAtOrigin::Local(const Similarity3& other,  ChartJacobian H) {
  Rotation r;  //Create a zero rotation to do the retraction
  Vector7 v;
  v.head<3>() = r.localCoordinates(other.R_);
  v.segment<3>(3) = other.t_.vector();
  //v.segment<3>(3) = translation().localCoordinates(other.translation());
  v[6] = other.s_ - 1.0;
  return v;
}
}


