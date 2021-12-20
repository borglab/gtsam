/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3.cpp
 * @brief   Rotation, common code between Rotation matrix and Quaternion
 * @author  Alireza Fathi
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Varun Agrawal
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <boost/math/constants/constants.hpp>

#include <cmath>
#include <random>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void Rot3::print(const std::string& s) const {
  cout << (s.empty() ? "R: " : s + " ");
  gtsam::print(static_cast<Matrix>(matrix()));
}

/* ************************************************************************* */
Rot3 Rot3::Random(std::mt19937& rng) {
  Unit3 axis = Unit3::Random(rng);
  uniform_real_distribution<double> randomAngle(-M_PI, M_PI);
  double angle = randomAngle(rng);
  return AxisAngle(axis, angle);
}



/* ************************************************************************* */
Rot3 Rot3::AlignPair(const Unit3& axis, const Unit3& a_p, const Unit3& b_p) {
  // if a_p is already aligned with b_p, return the identity rotation
  if (std::abs(a_p.dot(b_p)) > 0.999999999) {
    return Rot3();
  }

  // Check axis was not degenerate cross product
  const Vector3 z = axis.unitVector();
  if (z.hasNaN())
    throw std::runtime_error("AlignSinglePair: axis has Nans");

  // Now, calculate rotation that takes b_p to a_p
  const Matrix3 P = I_3x3 - z * z.transpose();  // orthogonal projector
  const Vector3 a_po = P * a_p.unitVector();    // point in a orthogonal to axis
  const Vector3 b_po = P * b_p.unitVector();    // point in b orthogonal to axis
  const Vector3 x = a_po.normalized();          // x-axis in axis-orthogonal plane, along a_p vector
  const Vector3 y = z.cross(x);                 // y-axis in axis-orthogonal plane
  const double u = x.dot(b_po);                 // x-coordinate for b_po
  const double v = y.dot(b_po);                 // y-coordinate for b_po
  double angle = std::atan2(v, u);
  return Rot3::AxisAngle(z, -angle);
}

/* ************************************************************************* */
Rot3 Rot3::AlignTwoPairs(const Unit3& a_p, const Unit3& b_p,  //
                         const Unit3& a_q, const Unit3& b_q) {
  // there are three frames in play:
  // a: the first frame in which p and q are measured
  // b: the second frame in which p and q are measured
  // i: intermediate, after aligning first pair

  // First, find rotation around that aligns a_p and b_p
  Rot3 i_R_b = AlignPair(a_p.cross(b_p), a_p, b_p);

  // Rotate points in frame b to the intermediate frame,
  // in which we expect the point p to be aligned now
  Unit3 i_q = i_R_b * b_q;
  assert(assert_equal(a_p, i_R_b * b_p, 1e-6));

  // Now align second pair: we need to align i_q to a_q
  Rot3 a_R_i = AlignPair(a_p, a_q, i_q);
  assert(assert_equal(a_p, a_R_i * a_p, 1e-6));
  assert(assert_equal(a_q, a_R_i * i_q, 1e-6));

  // The desired rotation is the product of both
  Rot3 a_R_b = a_R_i * i_R_b;
  return a_R_b;
}

/* ************************************************************************* */
bool Rot3::equals(const Rot3 & R, double tol) const {
  return equal_with_abs_tol(matrix(), R.matrix(), tol);
}

/* ************************************************************************* */
Point3 Rot3::operator*(const Point3& p) const {
  return rotate(p);
}

/* ************************************************************************* */
Unit3 Rot3::rotate(const Unit3& p,
    OptionalJacobian<2,3> HR, OptionalJacobian<2,2> Hp) const {
  Matrix32 Dp;
  Unit3 q = Unit3(rotate(p.point3(Hp ? &Dp : 0)));
  if (Hp) *Hp = q.basis().transpose() * matrix() * Dp;
  if (HR) *HR = -q.basis().transpose() * matrix() * p.skew();
  return q;
}

/* ************************************************************************* */
Unit3 Rot3::unrotate(const Unit3& p,
    OptionalJacobian<2,3> HR, OptionalJacobian<2,2> Hp) const {
  Matrix32 Dp;
  Unit3 q = Unit3(unrotate(p.point3(Dp)));
  if (Hp) *Hp = q.basis().transpose() * matrix().transpose () * Dp;
  if (HR) *HR = q.basis().transpose() * q.skew();
  return q;
}

/* ************************************************************************* */
Unit3 Rot3::operator*(const Unit3& p) const {
  return rotate(p);
}

/* ************************************************************************* */
// see doc/math.lyx, SO(3) section
Point3 Rot3::unrotate(const Point3& p, OptionalJacobian<3,3> H1,
    OptionalJacobian<3,3> H2) const {
  const Matrix3& Rt = transpose();
  Point3 q(Rt * p); // q = Rt*p
  const double wx = q.x(), wy = q.y(), wz = q.z();
  if (H1)
    *H1 << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0;
  if (H2)
    *H2 = Rt;
  return q;
}

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
Vector3 Rot3::xyz(OptionalJacobian<3, 3> H) const {
  Matrix3 I;Vector3 q;
  if (H) {
    Matrix93 mH;
    const auto m = matrix();
#ifdef GTSAM_USE_QUATERNIONS
    SO3{m}.vec(mH);
#else
    rot_.vec(mH);
#endif

    Matrix39 qHm;
    boost::tie(I, q) = RQ(m, qHm);

    // TODO : Explore whether this expression can be optimized as both
    // qHm and mH are super-sparse
    *H = qHm * mH;
  } else
    boost::tie(I, q) = RQ(matrix());
  return q;
}

/* ************************************************************************* */
Vector3 Rot3::ypr(OptionalJacobian<3, 3> H) const {
  Vector3 q = xyz(H);
  if (H) H->row(0).swap(H->row(2));

  return Vector3(q(2),q(1),q(0));
}

/* ************************************************************************* */
Vector3 Rot3::rpy(OptionalJacobian<3, 3> H) const { return xyz(H); }

/* ************************************************************************* */
double Rot3::roll(OptionalJacobian<1, 3> H) const {
  double r;
  if (H) {
    Matrix3 xyzH;
    r = xyz(xyzH)(0);
    *H = xyzH.row(0);
  } else
    r = xyz()(0);
  return r;
}

/* ************************************************************************* */
double Rot3::pitch(OptionalJacobian<1, 3> H) const {
  double p;
  if (H) {
    Matrix3 xyzH;
    p = xyz(xyzH)(1);
    *H = xyzH.row(1);
  } else
    p = xyz()(1);
  return p;
}

/* ************************************************************************* */
double Rot3::yaw(OptionalJacobian<1, 3> H) const {
  double y;
  if (H) {
    Matrix3 xyzH;
    y = xyz(xyzH)(2);
    *H = xyzH.row(2);
  } else
    y = xyz()(2);
  return y;
}

/* ************************************************************************* */
Vector Rot3::quaternion() const {
  gtsam::Quaternion q = toQuaternion();
  Vector v(4);
  v(0) = q.w();
  v(1) = q.x();
  v(2) = q.y();
  v(3) = q.z();
  return v;
}

/* ************************************************************************* */
pair<Unit3, double> Rot3::axisAngle() const {
  const Vector3 omega = Rot3::Logmap(*this);
  return std::pair<Unit3, double>(Unit3(omega), omega.norm());
}

/* ************************************************************************* */
Matrix3 Rot3::ExpmapDerivative(const Vector3& x) {
  return SO3::ExpmapDerivative(x);
}

/* ************************************************************************* */
Matrix3 Rot3::LogmapDerivative(const Vector3& x)    {
  return SO3::LogmapDerivative(x);
}

/* ************************************************************************* */
pair<Matrix3, Vector3> RQ(const Matrix3& A, OptionalJacobian<3, 9> H) {
  const double x = -atan2(-A(2, 1), A(2, 2));
  const auto Qx = Rot3::Rx(-x).matrix();
  const Matrix3 B = A * Qx;

  const double y = -atan2(B(2, 0), B(2, 2));
  const auto Qy = Rot3::Ry(-y).matrix();
  const Matrix3 C = B * Qy;

  const double z = -atan2(-C(1, 0), C(1, 1));
  const auto Qz = Rot3::Rz(-z).matrix();
  const Matrix3 R = C * Qz;

  if (H) {
    if (std::abs(y - M_PI / 2) < 1e-2)
      throw std::runtime_error(
          "Rot3::RQ : Derivative undefined at singularity (gimbal lock)");

    auto atan_d1 = [](double y, double x) { return x / (x * x + y * y); };
    auto atan_d2 = [](double y, double x) { return -y / (x * x + y * y); };

    const auto sx = -Qx(2, 1), cx = Qx(1, 1);
    const auto sy = -Qy(0, 2), cy = Qy(0, 0);

    *H = Matrix39::Zero();
    // First, calculate the derivate of x
    (*H)(0, 5) = atan_d1(A(2, 1), A(2, 2));
    (*H)(0, 8) = atan_d2(A(2, 1), A(2, 2));

    // Next, calculate the derivate of y. We have
    // b20 = a20 and b22 = a21 * sx + a22 * cx
    (*H)(1, 2) = -atan_d1(B(2, 0), B(2, 2));
    const auto yHb22 = -atan_d2(B(2, 0), B(2, 2));
    (*H)(1, 5) = yHb22 * sx;
    (*H)(1, 8) = yHb22 * cx;

    // Next, calculate the derivate of z. We have
    // c20 = a10 * cy + a11 * sx * sy + a12 * cx * sy
    // c22 = a11 * cx - a12 * sx
    const auto c10Hx = (A(1, 1) * cx - A(1, 2) * sx) * sy;
    const auto c10Hy = A(1, 2) * cx * cy + A(1, 1) * cy * sx - A(1, 0) * sy;
    Vector9 c10HA = c10Hx * H->row(0) + c10Hy * H->row(1);
    c10HA[1] = cy;
    c10HA[4] = sx * sy;
    c10HA[7] = cx * sy;

    const auto c11Hx = -A(1, 2) * cx - A(1, 1) * sx;
    Vector9 c11HA = c11Hx * H->row(0);
    c11HA[4] = cx;
    c11HA[7] = -sx;

    H->block<1, 9>(2, 0) =
        atan_d1(C(1, 0), C(1, 1)) * c10HA + atan_d2(C(1, 0), C(1, 1)) * c11HA;
  }

  const auto xyz = Vector3(x, y, z);
  return make_pair(R, xyz);
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const Rot3& R) {
  os << R.matrix().format(matlabFormat());
  return os;
}

/* ************************************************************************* */
Rot3 Rot3::slerp(double t, const Rot3& other) const {
  return interpolate(*this, other, t);
}

/* ************************************************************************* */

} // namespace gtsam

