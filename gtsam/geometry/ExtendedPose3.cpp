/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file  ExtendedPose3.cpp
 * @brief 3D ExtendedPose3
 * @author Martin Brossard
 */

#include <gtsam/base/concepts.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/concepts.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace std;

namespace gtsam {

/** instantiate concept checks */
GTSAM_CONCEPT_POSE_INST(ExtendedPose3);

/* ************************************************************************* */
ExtendedPose3 ExtendedPose3::Create(const Rot3& R, const Point3& v, const Point3& p, OptionalJacobian<9, 3> HR,
                                    OptionalJacobian<9, 3> Hv, OptionalJacobian<9, 3> Hp) {
  if (HR)
    *HR << I_3x3, Z_3x3, Z_3x3;
  if (Hv)
    *Hv << Z_3x3, R.transpose(), Z_3x3;
  if (Hp)
    *Hp << Z_3x3, Z_3x3, R.transpose();
  return ExtendedPose3(R, v, p);
}

/* ************************************************************************* */
ExtendedPose3 ExtendedPose3::inverse() const {
  Rot3 Rt = R_.inverse();
  return ExtendedPose3(Rt, Rt * (-v_), Rt * (-p_));
}

/* ************************************************************************* */
// Calculate Adjoint map
// Ad_pose is 9*9 matrix that when applied to twist xi, returns Ad_pose(xi)
Matrix9 ExtendedPose3::AdjointMap() const {
  const Matrix3 R = R_.matrix();
  Matrix3 A = skewSymmetric(v_.x(), v_.y(), v_.z()) * R;
  Matrix3 B = skewSymmetric(p_.x(), p_.y(), p_.z()) * R;
  Matrix9 adj;
  adj << R, Z_3x3, Z_3x3, A, R, Z_3x3, B, Z_3x3, R;
  return adj;
}

/* ************************************************************************* */
Matrix9 ExtendedPose3::adjointMap(const Vector9& xi) {
  Matrix3 w_hat = skewSymmetric(xi(0), xi(1), xi(2));
  Matrix3 v_hat = skewSymmetric(xi(3), xi(4), xi(5));
  Matrix3 a_hat = skewSymmetric(xi(6), xi(7), xi(8));
  Matrix9 adj;
  adj << w_hat, Z_3x3, Z_3x3, v_hat, w_hat, Z_3x3, a_hat, Z_3x3, w_hat;
  return adj;
}

/* ************************************************************************* */
Vector9 ExtendedPose3::adjoint(const Vector9& xi, const Vector9& y, OptionalJacobian<9, 9> Hxi) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 9; ++i) {
      Vector9 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix9 Gi = adjointMap(dxi);
      Hxi->col(i) = Gi * y;
    }
  }
  return adjointMap(xi) * y;
}

/* ************************************************************************* */
Vector9 ExtendedPose3::adjointTranspose(const Vector9& xi, const Vector9& y, OptionalJacobian<9, 9> Hxi) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 9; ++i) {
      Vector9 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix9 GTi = adjointMap(dxi).transpose();
      Hxi->col(i) = GTi * y;
    }
  }
  return adjointMap(xi).transpose() * y;
}

/* ************************************************************************* */
void ExtendedPose3::print(const string& s) const {
  cout << s;
  R_.print("R:\n");
  cout << "v:" << v_ << ";" << endl;
  cout << "p:" << p_ << ";" << endl;
}

/* ************************************************************************* */
bool ExtendedPose3::equals(const ExtendedPose3& pose, double tol) const {
  return R_.equals(pose.R_, tol) && traits<Point3>::Equals(v_, pose.v_, tol) &&
         traits<Point3>::Equals(p_, pose.p_, tol);
}

/* ************************************************************************* */
ExtendedPose3 ExtendedPose3::Expmap(const Vector9& xi, OptionalJacobian<9, 9> Hxi) {
  if (Hxi)
    *Hxi = ExpmapDerivative(xi);

  Vector3 phi(xi(0), xi(1), xi(2)), nu(xi(3), xi(4), xi(5)), rho(xi(6), xi(7), xi(8));

  Rot3 R = Rot3::Expmap(phi);

  double phi_norm = phi.norm();
  if (phi_norm < 1e-8)
    return ExtendedPose3(Rot3(), Point3(nu), Point3(rho));
  else {
    Matrix W = skewSymmetric(phi / phi_norm);
    Matrix A = I_3x3 + ((1 - cos(phi_norm)) / phi_norm) * W + ((phi_norm - sin(phi_norm)) / phi_norm) * (W * W);
    return ExtendedPose3(Rot3::Expmap(phi), Point3(A * nu), Point3(A * rho));
  }
}

/* ************************************************************************* */
Vector9 ExtendedPose3::Logmap(const ExtendedPose3& pose, OptionalJacobian<9, 9> Hpose) {
  if (Hpose)
    *Hpose = LogmapDerivative(pose);
  const Vector3 phi = Rot3::Logmap(pose.rotation());
  const Vector3& v = pose.velocity();
  const Vector3& p = pose.position();
  const double t = phi.norm();
  if (t < 1e-8) {
    Vector9 log;
    log << phi, v, p;
    return log;
  } else {
    const Matrix3 W = skewSymmetric(phi / t);

    const double Tan = tan(0.5 * t);
    const Vector3 Wp = W * p;
    const Vector3 Wv = W * v;
    const Vector3 nu = v - (0.5 * t) * Wv + (1 - t / (2. * Tan)) * (W * Wv);
    const Vector3 rho = p - (0.5 * t) * Wp + (1 - t / (2. * Tan)) * (W * Wp);
    Vector9 log;
    log << phi, nu, rho;
    return log;
  }
}

/* ************************************************************************* */
ExtendedPose3 ExtendedPose3::ChartAtOrigin::Retract(const Vector9& xi, ChartJacobian Hxi) {
  return Expmap(xi, Hxi);
}

/* ************************************************************************* */
Vector9 ExtendedPose3::ChartAtOrigin::Local(const ExtendedPose3& pose, ChartJacobian Hpose) {
  return Logmap(pose, Hpose);
}

/* ************************************************************************* */
Vector9 ExtendedPose3::boxminus(const ExtendedPose3& g) const {
  // Matrix3 D_dR_R, D_dt_R, D_dv_R;
  // const Rot3 dR = R_.between(g.R_, H1 ? &D_dR_R : 0);
  // const Point3 dt = R_.unrotate(g.p_ - p_, H1 ? &D_dt_R : 0);
  // const Vector dv = R_.unrotate(g.v_ - v_, H1 ? &D_dv_R : 0);

  // Vector9 xi;
  // Matrix3 D_xi_R;
  // xi << Rot3::Logmap(dR, (H1 || H2) ? &D_xi_R : 0), dt, dv;
  // if (H1) {
  //   *H1 << D_xi_R * D_dR_R, Z_3x3, Z_3x3, //
  //   D_dt_R, -I_3x3, Z_3x3, //
  //   D_dv_R, Z_3x3, -I_3x3;
  // }
  // if (H2) {
  //   *H2 << D_xi_R, Z_3x3, Z_3x3, //
  //   Z_3x3, dR.matrix(), Z_3x3, //
  //   Z_3x3, Z_3x3, dR.matrix();
  // }

  Matrix3 D_dR_R, D_dt_R, D_dv_R;
  const Rot3 dR = g.R_;
  const Point3 dt = g.p_;
  const Vector dv = g.v_;

  Vector9 xi;
  xi << Rot3::Logmap(dR), dt, dv;
  return xi;
}

/* ************************************************************************* */
/**
 * Compute the 6x3 bottom-left block Qs of the SE_2(3) Expmap derivative matrix
 */
static Matrix63 computeQforExpmapDerivative(const Vector9& xi) {
  const auto omega = xi.head<3>();
  const auto nu = xi.segment<3>(3);
  const auto rho = xi.tail<3>();
  const Matrix3 V = skewSymmetric(nu);
  const Matrix3 P = skewSymmetric(rho);
  const Matrix3 W = skewSymmetric(omega);

  Matrix3 Qv, Qp;
  Matrix63 Q;

  // The closed-form formula in Barfoot14tro eq. (102)
  double phi = omega.norm();
  if (std::abs(phi) > 1e-5) {
    const double sinPhi = sin(phi), cosPhi = cos(phi);
    const double phi2 = phi * phi, phi3 = phi2 * phi, phi4 = phi3 * phi, phi5 = phi4 * phi;
    // Invert the sign of odd-order terms to have the right Jacobian
    Qv = -0.5 * V + (phi - sinPhi) / phi3 * (W * V + V * W - W * V * W) +
         (1 - phi2 / 2 - cosPhi) / phi4 * (W * W * V + V * W * W - 3 * W * V * W) -
         0.5 * ((1 - phi2 / 2 - cosPhi) / phi4 - 3 * (phi - sinPhi - phi3 / 6.) / phi5) *
             (W * V * W * W + W * W * V * W);
    Qp = -0.5 * P + (phi - sinPhi) / phi3 * (W * P + P * W - W * P * W) +
         (1 - phi2 / 2 - cosPhi) / phi4 * (W * W * P + P * W * W - 3 * W * P * W) -
         0.5 * ((1 - phi2 / 2 - cosPhi) / phi4 - 3 * (phi - sinPhi - phi3 / 6.) / phi5) *
             (W * P * W * W + W * W * P * W);
  } else {
    Qv = -0.5 * V + 1. / 6. * (W * V + V * W - W * V * W) + 1. / 24. * (W * W * V + V * W * W - 3 * W * V * W) -
         0.5 * (1. / 24. + 3. / 120.) * (W * V * W * W + W * W * V * W);
    Qp = -0.5 * P + 1. / 6. * (W * P + P * W - W * P * W) + 1. / 24. * (W * W * P + P * W * W - 3 * W * P * W) -
         0.5 * (1. / 24. + 3. / 120.) * (W * P * W * W + W * W * P * W);
  }

  Q << Qv, Qp;
  return Q;
}

/* ************************************************************************* */
Matrix9 ExtendedPose3::ExpmapDerivative(const Vector9& xi) {
  const Vector3 w = xi.head<3>();
  const Matrix3 Jw = Rot3::ExpmapDerivative(w);
  const Matrix63 Q = computeQforExpmapDerivative(xi);
  const Matrix3 Qv = Q.topRows<3>();
  const Matrix3 Qp = Q.bottomRows<3>();
  Matrix9 J;
  J << Jw, Z_3x3, Z_3x3, Qv, Jw, Z_3x3, Qp, Z_3x3, Jw;
  return J;
}

/* ************************************************************************* */
Matrix9 ExtendedPose3::LogmapDerivative(const ExtendedPose3& pose) {
  const Vector9 xi = Logmap(pose);
  const Vector3 w = xi.head<3>();
  const Matrix3 Jw = Rot3::LogmapDerivative(w);
  const Matrix63 Q = computeQforExpmapDerivative(xi);
  const Matrix3 Qv = Q.topRows<3>();
  const Matrix3 Qp = Q.bottomRows<3>();
  const Matrix3 Qv2 = -Jw * Qv * Jw;
  const Matrix3 Qp2 = -Jw * Qp * Jw;
  Matrix9 J;

  J << Jw, Z_3x3, Z_3x3, Qv2, Jw, Z_3x3, Qp2, Z_3x3, Jw;
  return J;
}

/* ************************************************************************* */
const Point3& ExtendedPose3::position(OptionalJacobian<3, 9> Hself) const {
  if (Hself)
    *Hself << Z_3x3, Z_3x3, rotation().matrix();
  return p_;
}

const Point3& ExtendedPose3::translation(OptionalJacobian<3, 9> Hself) const {
  if (Hself)
    *Hself << Z_3x3, Z_3x3, rotation().matrix();
  return p_;
}

/* ************************************************************************* */

const Point3& ExtendedPose3::velocity(OptionalJacobian<3, 9> Hself) const {
  if (Hself) {
    *Hself << Z_3x3, rotation().matrix(), Z_3x3;
  }
  return v_;
}

/* ************************************************************************* */

const Rot3& ExtendedPose3::rotation(OptionalJacobian<3, 9> Hself) const {
  if (Hself) {
    *Hself << I_3x3, Z_3x3, Z_3x3;
  }
  return R_;
}

/* ************************************************************************* */
Matrix5 ExtendedPose3::matrix() const {
  Matrix5 mat;
  mat << R_.matrix(), v_, p_, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  return mat;
}

/* ************************************************************************* */
ExtendedPose3 ExtendedPose3::transformPoseFrom(const ExtendedPose3& aTb, OptionalJacobian<9, 9> Hself,
                                               OptionalJacobian<9, 9> HaTb) const {
  const ExtendedPose3& wTa = *this;
  return wTa.compose(aTb, Hself, HaTb);
}

/* ************************************************************************* */
ExtendedPose3 ExtendedPose3::transformPoseTo(const ExtendedPose3& wTb, OptionalJacobian<9, 9> Hself,
                                             OptionalJacobian<9, 9> HwTb) const {
  if (Hself)
    *Hself = -wTb.inverse().AdjointMap() * AdjointMap();
  if (HwTb)
    *HwTb = I_9x9;
  const ExtendedPose3& wTa = *this;
  return wTa.inverse() * wTb;
}

/* ************************************************************************* */
Point3 ExtendedPose3::transformFrom(const Point3& point, OptionalJacobian<3, 9> Hself,
                                    OptionalJacobian<3, 3> Hpoint) const {
  // Only get matrix once, to avoid multiple allocations,
  // as well as multiple conversions in the Quaternion case
  const Matrix3 R = R_.matrix();
  if (Hself) {
    Hself->leftCols<3>() = R * skewSymmetric(-point.x(), -point.y(), -point.z());
    Hself->rightCols<3>() = R;
  }
  if (Hpoint) {
    *Hpoint = R;
  }
  return R_ * point + p_;
}

/* ************************************************************************* */
Point3 ExtendedPose3::transformTo(const Point3& point, OptionalJacobian<3, 9> Hself,
                                  OptionalJacobian<3, 3> Hpoint) const {
  // Only get transpose once, to avoid multiple allocations,
  // as well as multiple conversions in the Quaternion case
  const Matrix3 Rt = R_.transpose();
  Point3 q(Rt * (point - p_));
  if (Hself) {
    const double wx = q.x(), wy = q.y(), wz = q.z();
    (*Hself) << 0.0, -wz, +wy, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, +wz, 0.0, -wx, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -wy, +wx,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0;
  }
  if (Hpoint) {
    *Hpoint = Rt;
  }
  return q;
}

/* ************************************************************************* */
double ExtendedPose3::range(const Point3& point, OptionalJacobian<1, 9> Hself, OptionalJacobian<1, 3> Hpoint) const {
  Matrix39 D_local_pose;
  Matrix3 D_local_point;
  Point3 local = transformTo(point, Hself ? &D_local_pose : 0, Hpoint ? &D_local_point : 0);
  if (!Hself && !Hpoint) {
    return local.norm();
  } else {
    Matrix13 D_r_local;
    const double r = norm3(local, D_r_local);
    if (Hself)
      *Hself = D_r_local * D_local_pose;
    if (Hpoint)
      *Hpoint = D_r_local * D_local_point;
    return r;
  }
}

/* ************************************************************************* */
double ExtendedPose3::range(const ExtendedPose3& pose, OptionalJacobian<1, 9> Hself,
                            OptionalJacobian<1, 9> Hpose) const {
  Matrix13 D_local_point;
  double r = range(pose.translation(), Hself, Hpose ? &D_local_point : 0);
  if (Hpose)
    *Hpose << Matrix13::Zero(), D_local_point * pose.rotation().matrix();
  return r;
}

/* ************************************************************************* */
Unit3 ExtendedPose3::bearing(const Point3& point, OptionalJacobian<2, 9> Hself, OptionalJacobian<2, 3> Hpoint) const {
  Matrix39 D_local_pose;
  Matrix3 D_local_point;
  Point3 local = transformTo(point, Hself ? &D_local_pose : 0, Hpoint ? &D_local_point : 0);
  if (!Hself && !Hpoint) {
    return Unit3(local);
  } else {
    Matrix23 D_b_local;
    Unit3 b = Unit3::FromPoint3(local, D_b_local);
    if (Hself)
      *Hself = D_b_local * D_local_pose;
    if (Hpoint)
      *Hpoint = D_b_local * D_local_point;
    return b;
  }
}

/* ************************************************************************* */
Unit3 ExtendedPose3::bearing(const ExtendedPose3& pose, OptionalJacobian<2, 9> Hself,
                             OptionalJacobian<2, 9> Hpose) const {
  if (Hpose) {
    Hpose->setZero();
    return bearing(pose.translation(), Hself, Hpose.cols<3>(3));
  }
  return bearing(pose.translation(), Hself, {});
}

/* ************************************************************************* */
std::optional<ExtendedPose3> ExtendedPose3::Align(const std::vector<Point3Pair>& abPointPairs) {
  const size_t n = abPointPairs.size();
  if (n < 3)
    return {};  // we need at least three pairs

  // calculate centroids
  Point3 aCentroid(0, 0, 0), bCentroid(0, 0, 0);
  for (const Point3Pair& abPair : abPointPairs) {
    aCentroid += abPair.first;
    bCentroid += abPair.second;
  }
  double f = 1.0 / n;
  aCentroid *= f;
  bCentroid *= f;

  // Add to form H matrix
  Matrix3 H = Z_3x3;
  for (const Point3Pair& abPair : abPointPairs) {
    Point3 da = abPair.first - aCentroid;
    Point3 db = abPair.second - bCentroid;
    H += da * db.transpose();
  }

  // ClosestTo finds rotation matrix closest to H in Frobenius sense
  Rot3 aRb = Rot3::ClosestTo(H);
  Point3 aTb = Point3(aCentroid) - aRb * Point3(bCentroid);
  Point3 v;
  return ExtendedPose3(aRb, v, aTb);
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const ExtendedPose3& pose) {
  os << pose.rotation();
  const Point3& v = pose.velocity();
  const Point3& p = pose.position();
  os << "v:[" << v.x() << ", " << v.y() << ", " << v.z() << "];\n";
  os << "p:[" << p.x() << ", " << p.y() << ", " << p.z() << "];\n";
  return os;
}

}  // namespace gtsam
