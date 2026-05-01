/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Pose3.cpp
 * @brief 3D Pose manifold SO(3) x R^3 and group SE(3)
 */

#include <gtsam/base/concepts.h>
#include <gtsam/geometry/Kernel.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/concepts.h>

#include <cmath>
#include <iostream>
#include <string>

namespace gtsam {

/** instantiate concept checks */
GTSAM_CONCEPT_POSE_INST(Pose3)

/* ************************************************************************* */
Pose3::Pose3(const Pose2& pose2)
    : Base(Rot3::Rodrigues(0, 0, pose2.theta()),
           Vector3(pose2.x(), pose2.y(), 0.0)) {}

/* ************************************************************************* */
Pose3 Pose3::Create(const Rot3& R, const Point3& t, OptionalJacobian<6, 3> HR,
                    OptionalJacobian<6, 3> Ht) {
  if (HR) *HR << I_3x3, Z_3x3;
  if (Ht) *Ht << Z_3x3, R.transpose();
  return Pose3(R, t);
}

// Pose2 constructor Jacobian is always the same.
static const Matrix63 Hpose2 = (Matrix63() << //
    0., 0., 0., //
    0., 0., 0.,//
    0., 0., 1.,//
    1., 0., 0.,//
    0., 1., 0.,//
    0., 0., 0.).finished();

Pose3 Pose3::FromPose2(const Pose2& p, OptionalJacobian<6, 3> H) {
  if (H) *H << Hpose2;
  return Pose3(p);
}

/* ************************************************************************* */
void Pose3::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

/* ************************************************************************* */
bool Pose3::equals(const Pose3& pose, double tol) const {
  return R_.equals(pose.R_, tol) && traits<Point3>::Equals(t_, pose.t_, tol);
}

/* ************************************************************************* */
Pose3 Pose3::interpolateRt(const Pose3& T, double t,
                           OptionalJacobian<6, 6> Hself,
                           OptionalJacobian<6, 6> Harg,
                           OptionalJacobian<6, 1> Ht) const {
  if(Hself || Harg || Ht){
    typename MakeJacobian<Rot3, Rot3>::type HselfRot, HargRot;
    typename MakeJacobian<Rot3, double>::type HtRot;
    typename MakeJacobian<Point3, Point3>::type HselfPoint, HargPoint;
    typename MakeJacobian<Point3, double>::type HtPoint;

    Rot3 Rint = interpolate<Rot3>(R_, T.R_, t, HselfRot, HargRot, HtRot);
    Point3 Pint = interpolate<Point3>(t_, T.t_, t, HselfPoint, HargPoint, HtPoint);
    Pose3 result = Pose3(Rint, Pint);

    if(Hself) *Hself << HselfRot, Z_3x3, Z_3x3, Rint.transpose() * R_.matrix() * HselfPoint;
    if(Harg) *Harg << HargRot, Z_3x3, Z_3x3, Rint.transpose() * T.R_.matrix() * HargPoint;
    if(Ht) *Ht << HtRot, Rint.transpose() * HtPoint;

    return result;
  }
  return Pose3(interpolate<Rot3>(R_, T.R_, t),
               interpolate<Point3>(t_, T.t_, t));
}

/* ************************************************************************* */
// Expmap is implemented in so3::ExpmapFunctor::expmap, based on Ethan Eade's
// elegant Lie group document, at https://www.ethaneade.org/lie.pdf.
// See also [this document](doc/Jacobians.md)
Pose3 Pose3::Expmap(const Vector6& xi, OptionalJacobian<6, 6> Hxi) {
  // Get angular velocity omega and translational velocity v from twist xi
  const Vector3 w = xi.head<3>(), v = xi.tail<3>();

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  // Compute rotation using Expmap
#ifdef GTSAM_USE_QUATERNIONS
  const Rot3 R = traits<gtsam::Quaternion>::Expmap(w);
#else
  const Rot3 R(local.expmap());
#endif

  // The translation t = local.Jacobian().left() * v.
  // Here we call local.Jacobian().applyLeft, which is faster if you don't need
  // Jacobians, and returns Jacobian of t with respect to w if asked.
  // NOTE(Frank): this does the same as the intuitive formulas:
  //   t_parallel = w * w.dot(v);  // translation parallel to axis
  //   w_cross_v = w.cross(v);     // translation orthogonal to axis
  //   t = (w_cross_v - Rot3::Expmap(w) * w_cross_v + t_parallel) / theta2;
  // but Local does not need R, deals automatically with the case where theta2
  // is near zero, and also gives us the machinery for the Jacobians.
  Matrix3 H;
  const Vector3 t = local.Jacobian().applyLeft(v, Hxi ? &H : nullptr);

  if (Hxi) {
    // The Jacobian of expmap is given by the right Jacobian of SO(3):
    const Matrix3 Jr = local.Jacobian().right();
    // Chain H with R^T, the Jacobian of Pose3::Create with respect to t.
    const Matrix3 Rt = R.transpose();
    *Hxi << Jr, Z_3x3,  // Jr here *is* the Jacobian of expmap
        Rt * H, Jr;     // Jr = R^T * Jl, with Jl Jacobian of t in v.
  }

  return Pose3(R, t);
}

/* ************************************************************************* */
Pose3 Pose3::ChartAtOrigin::Retract(const Vector6& xi, ChartJacobian Hxi) {
#ifdef GTSAM_POSE3_EXPMAP
  return Expmap(xi, Hxi);
#else
  Matrix3 DR;
  const Rot3 R = Rot3::Retract(xi.head<3>(), Hxi ? &DR : nullptr);
  if (Hxi) {
    Hxi->setIdentity();
    Hxi->block<3, 3>(0, 0) = DR;
  }
  return Pose3(R, xi.tail<3>());
#endif
}

/* ************************************************************************* */
Vector6 Pose3::ChartAtOrigin::Local(const Pose3& pose, ChartJacobian Hpose) {
#ifdef GTSAM_POSE3_EXPMAP
  return Logmap(pose, Hpose);
#else
  Matrix3 DR;
  Vector6 xi;
  xi.head<3>() = Rot3::LocalCoordinates(pose.rotation(), Hpose ? &DR : nullptr);
  xi.tail<3>() = pose.translation();
  if (Hpose) {
    Hpose->setIdentity();
    Hpose->block<3, 3>(0, 0) = DR;
  }
  return xi;
#endif
}

/* ************************************************************************* */
const Point3& Pose3::translation(OptionalJacobian<3, 6> Hself) const {
  if (Hself) *Hself << Z_3x3, rotation().matrix();
  return t_;
}

/* ************************************************************************* */
Pose3 Pose3::transformPoseFrom(const Pose3& aTb, OptionalJacobian<6, 6> Hself,
                               OptionalJacobian<6, 6> HaTb) const {
  const Pose3& wTa = *this;
  return wTa.compose(aTb, Hself, HaTb);
}

/* ************************************************************************* */
Pose3 Pose3::transformPoseTo(const Pose3& wTb, OptionalJacobian<6, 6> Hself,
                             OptionalJacobian<6, 6> HwTb) const {
  if (Hself) *Hself = -wTb.inverse().AdjointMap() * AdjointMap();
  if (HwTb) *HwTb = I_6x6;
  const Pose3& wTa = *this;
  return wTa.inverse() * wTb;
}

/* ************************************************************************* */
Point3 Pose3::transformFrom(const Point3& point, OptionalJacobian<3, 6> Hself,
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
  return R_ * point + t_;
}

Matrix Pose3::transformFrom(const Matrix& points) const {
  if (points.rows() != 3) {
    throw std::invalid_argument("Pose3:transformFrom expects 3*N matrix.");
  }
  const Matrix3 R = R_.matrix();
  return (R * points).colwise() + t_;  // Eigen broadcasting!
}

/* ************************************************************************* */
Point3 Pose3::transformTo(const Point3& point, OptionalJacobian<3, 6> Hself,
                          OptionalJacobian<3, 3> Hpoint) const {
  // Only get transpose once, to avoid multiple allocations,
  // as well as multiple conversions in the Quaternion case
  const Matrix3 Rt = R_.transpose();
  const Point3 q(Rt*(point - t_));
  if (Hself) {
    const double wx = q.x(), wy = q.y(), wz = q.z();
    (*Hself) <<
        0.0, -wz, +wy,-1.0, 0.0, 0.0,
        +wz, 0.0, -wx, 0.0,-1.0, 0.0,
        -wy, +wx, 0.0, 0.0, 0.0,-1.0;
  }
  if (Hpoint) {
    *Hpoint = Rt;
  }
  return q;
}

Matrix Pose3::transformTo(const Matrix& points) const {
  if (points.rows() != 3) {
    throw std::invalid_argument("Pose3:transformTo expects 3*N matrix.");
  }
  const Matrix3 Rt = R_.transpose();
  return Rt * (points.colwise() - t_);  // Eigen broadcasting!
}

/* ************************************************************************* */
double Pose3::range(const Point3& point, OptionalJacobian<1, 6> Hself,
                    OptionalJacobian<1, 3> Hpoint) const {
  const Vector3 delta = point - t_;
  if (!Hself && !Hpoint) return delta.norm();

  Matrix13 D_r_point;
  const double r = norm3(delta, D_r_point);

  if (Hpoint) *Hpoint = D_r_point;
  if (Hself) {
    // Range is rotation-invariant: ||R^T(p-t)|| = ||p-t||, so d(range)/d(rotation) = 0.
    Hself->leftCols<3>().setZero();
    // Translation coordinates are in the body frame: dt_world = R * dt_body.
    Hself->rightCols<3>() = -D_r_point * R_.matrix();
  }
  return r;
}

/* ************************************************************************* */
double Pose3::range(const Pose3& pose, OptionalJacobian<1, 6> Hself,
                    OptionalJacobian<1, 6> Hpose) const {
  const Vector3 delta = pose.t_ - t_;
  if (!Hself && !Hpose) return delta.norm();

  Matrix13 D_r_point;
  const double r = norm3(delta, D_r_point);

  if (Hself) {
    // Range depends only on translation: ||t2-t1||.
    Hself->leftCols<3>().setZero();
    // Translation coordinates are in the body frame: dt_world = R * dt_body.
    Hself->rightCols<3>() = -D_r_point * R_.matrix();
  }

  if (Hpose) {
    Hpose->leftCols<3>().setZero();
    // Translation coordinates are in the body frame: dt_world = R * dt_body.
    Hpose->rightCols<3>() = D_r_point * pose.R_.matrix();
  }

  return r;
}

/* ************************************************************************* */
Unit3 Pose3::bearing(const Point3& point, OptionalJacobian<2, 6> Hself,
                     OptionalJacobian<2, 3> Hpoint) const {
  const Matrix3 Rt = R_.transpose();
  const Point3 local(Rt * (point - t_));

  if (!Hself && !Hpoint) return Unit3(local);

  Matrix23 D_b_local;
  const Unit3 b = Unit3::FromPoint3(local, D_b_local);
  if (Hself) {
    Hself->leftCols<3>() = D_b_local * skewSymmetric(local.x(), local.y(), local.z());
    Hself->rightCols<3>() = -D_b_local;
  }
  if (Hpoint) {
    *Hpoint = D_b_local * Rt;
  }
  return b;
}

/* ************************************************************************* */
Unit3 Pose3::bearing(const Pose3& pose, OptionalJacobian<2, 6> Hself,
  OptionalJacobian<2, 6> Hpose) const {
  Matrix23 D_bearing_point;
  const Point3 point = pose.translation();
  const Unit3 b = bearing(point, Hself, Hpose ? &D_bearing_point : 0);
  if (Hpose) {
    Hpose->leftCols<3>().setZero();
    Hpose->rightCols<3>() = D_bearing_point * pose.rotation().matrix();
  }
  return b;
}

/* ************************************************************************* */
std::optional<Pose3> Pose3::Align(const Point3Pairs &abPointPairs) {
  const size_t n = abPointPairs.size();
  if (n < 3) {
    return {};  // we need at least three pairs
  }

  // calculate centroids
  const auto centroids = means(abPointPairs);

  // Add to form H matrix
  Matrix3 H = Z_3x3;
  for (const Point3Pair &abPair : abPointPairs) {
    const Point3 da = abPair.first - centroids.first;
    const Point3 db = abPair.second - centroids.second;
    H += da * db.transpose();
  }

  // ClosestTo finds rotation matrix closest to H in Frobenius sense
  const Rot3 aRb = Rot3::ClosestTo(H);
  const Point3 aTb = centroids.first - aRb * centroids.second;
  return Pose3(aRb, aTb);
}

std::optional<Pose3> Pose3::Align(const Matrix& a, const Matrix& b) {
  if (a.rows() != 3 || b.rows() != 3 || a.cols() != b.cols()) {
    throw std::invalid_argument(
        "Pose3:Align expects 3*N matrices of equal shape.");
  }
  Point3Pairs abPointPairs;
  for (Eigen::Index j = 0; j < a.cols(); j++) {
    abPointPairs.emplace_back(a.col(j), b.col(j));
  }
  return Pose3::Align(abPointPairs);
}

/* ************************************************************************* */
Pose3 Pose3::slerp(double t, const Pose3& other, OptionalJacobian<6, 6> Hx, OptionalJacobian<6, 6> Hy) const {
  return interpolate(*this, other, t, Hx, Hy);
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Pose3& pose) {
  // Both Rot3 and Point3 have ostream definitions so we use them.
  os << "R: " << pose.rotation() << "\n";
  os << "t: " << pose.translation().transpose();
  return os;
}

} // namespace gtsam
