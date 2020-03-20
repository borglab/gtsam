/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3M.cpp
 * @brief   Rotation (internal: 3*3 matrix representation*)
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/config.h> // Get GTSAM_USE_QUATERNIONS macro

#ifndef GTSAM_USE_QUATERNIONS

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Rot3::Rot3() : rot_(I_3x3) {}

/* ************************************************************************* */
Rot3::Rot3(const Point3& col1, const Point3& col2, const Point3& col3) {
  Matrix3 R;
  R << col1, col2, col3;
  rot_ = SO3(R);
}

/* ************************************************************************* */
Rot3::Rot3(double R11, double R12, double R13, double R21, double R22,
           double R23, double R31, double R32, double R33) {
  Matrix3 R;
  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
  rot_ = SO3(R);
}

/* ************************************************************************* */
Rot3::Rot3(const gtsam::Quaternion& q) : rot_(q.toRotationMatrix()) {
}

/* ************************************************************************* */
Rot3 Rot3::Rx(double t) {
  double st = sin(t), ct = cos(t);
  return Rot3(
      1,  0,  0,
      0, ct,-st,
      0, st, ct);
}

/* ************************************************************************* */
Rot3 Rot3::Ry(double t) {
  double st = sin(t), ct = cos(t);
  return Rot3(
      ct, 0, st,
      0, 1,  0,
      -st, 0, ct);
}

/* ************************************************************************* */
Rot3 Rot3::Rz(double t) {
  double st = sin(t), ct = cos(t);
  return Rot3(
      ct,-st, 0,
      st, ct, 0,
      0,  0, 1);
}

/* ************************************************************************* */
// Considerably faster than composing matrices above !
Rot3 Rot3::RzRyRx(double x, double y, double z) {
  double cx=cos(x),sx=sin(x);
  double cy=cos(y),sy=sin(y);
  double cz=cos(z),sz=sin(z);
  double ss_ = sx * sy;
  double cs_ = cx * sy;
  double sc_ = sx * cy;
  double cc_ = cx * cy;
  double c_s = cx * sz;
  double s_s = sx * sz;
  double _cs = cy * sz;
  double _cc = cy * cz;
  double s_c = sx * cz;
  double c_c = cx * cz;
  double ssc = ss_ * cz, csc = cs_ * cz, sss = ss_ * sz, css = cs_ * sz;
  return Rot3(
      _cc,- c_s + ssc,  s_s + csc,
      _cs,  c_c + sss, -s_c + css,
      -sy,        sc_,        cc_
  );
}

/* ************************************************************************* */
Rot3 Rot3::operator*(const Rot3& R2) const {
  return Rot3(rot_*R2.rot_);
}

/* ************************************************************************* */
Matrix3 Rot3::transpose() const {
  return rot_.matrix().transpose();
}

/* ************************************************************************* */
Point3 Rot3::rotate(const Point3& p,
    OptionalJacobian<3,3> H1,  OptionalJacobian<3,3> H2) const {
  if (H1) *H1 = rot_.matrix() * skewSymmetric(-p.x(), -p.y(), -p.z());
  if (H2) *H2 = rot_.matrix();
  return rot_.matrix() * p;
}

/* ************************************************************************* */
// Log map at identity - return the canonical coordinates of this rotation
Vector3 Rot3::Logmap(const Rot3& R, OptionalJacobian<3,3> H) {
  return SO3::Logmap(R.rot_,H);
}

/* ************************************************************************* */
Rot3 Rot3::CayleyChart::Retract(const Vector3& omega, OptionalJacobian<3,3> H) {
  if (H) throw std::runtime_error("Rot3::CayleyChart::Retract Derivative");
  const double x = omega(0), y = omega(1), z = omega(2);
  const double x2 = x * x, y2 = y * y, z2 = z * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  const double f = 1.0 / (4.0 + x2 + y2 + z2), _2f = 2.0 * f;
  return Rot3((4 + x2 - y2 - z2) * f, (xy - 2 * z) * _2f, (xz + 2 * y) * _2f,
          (xy + 2 * z) * _2f, (4 - x2 + y2 - z2) * f, (yz - 2 * x) * _2f,
          (xz - 2 * y) * _2f, (yz + 2 * x) * _2f, (4 - x2 - y2 + z2) * f);
}

/* ************************************************************************* */
Vector3 Rot3::CayleyChart::Local(const Rot3& R, OptionalJacobian<3,3> H) {
  if (H) throw std::runtime_error("Rot3::CayleyChart::Local Derivative");
  // Create a fixed-size matrix
  Matrix3 A = R.matrix();
  // Mathematica closed form optimization (procrastination?) gone wild:
  const double a = A(0, 0), b = A(0, 1), c = A(0, 2);
  const double d = A(1, 0), e = A(1, 1), f = A(1, 2);
  const double g = A(2, 0), h = A(2, 1), i = A(2, 2);
  const double di = d * i, ce = c * e, cd = c * d, fg = f * g;
  const double M = 1 + e - f * h + i + e * i;
  const double K = -4.0 / (cd * h + M + a * M - g * (c + ce) - b * (d + di - fg));
  const double x = a * f - cd + f;
  const double y = b * f - ce - c;
  const double z = fg - di - d;
  return K * Vector3(x, y, z);
}

/* ************************************************************************* */
Rot3 Rot3::ChartAtOrigin::Retract(const Vector3& omega, ChartJacobian H) {
  static const CoordinatesMode mode = ROT3_DEFAULT_COORDINATES_MODE;
  if (mode == Rot3::EXPMAP) return Expmap(omega, H);
  if (mode == Rot3::CAYLEY) return CayleyChart::Retract(omega, H);
  else throw std::runtime_error("Rot3::Retract: unknown mode");
}

/* ************************************************************************* */
Vector3 Rot3::ChartAtOrigin::Local(const Rot3& R, ChartJacobian H) {
  static const CoordinatesMode mode = ROT3_DEFAULT_COORDINATES_MODE;
  if (mode == Rot3::EXPMAP) return Logmap(R, H);
  if (mode == Rot3::CAYLEY) return CayleyChart::Local(R, H);
  else throw std::runtime_error("Rot3::Local: unknown mode");
}

/* ************************************************************************* */
Matrix3 Rot3::matrix() const {
  return rot_.matrix();
}

/* ************************************************************************* */
Point3 Rot3::r1() const { return Point3(rot_.matrix().col(0)); }

/* ************************************************************************* */
Point3 Rot3::r2() const { return Point3(rot_.matrix().col(1)); }

/* ************************************************************************* */
Point3 Rot3::r3() const { return Point3(rot_.matrix().col(2)); }

/* ************************************************************************* */
gtsam::Quaternion Rot3::toQuaternion() const {
  return gtsam::Quaternion(rot_.matrix());
}

/* ************************************************************************* */

} // namespace gtsam

#endif
