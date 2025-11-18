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
Pose3::Pose3(const Pose2& pose2) :
    R_(Rot3::Rodrigues(0, 0, pose2.theta())), t_(
        Point3(pose2.x(), pose2.y(), 0)) {
}

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
Pose3 Pose3::inverse() const {
  Rot3 Rt = R_.inverse();
  return Pose3(Rt, Rt * (-t_));
}

/* ************************************************************************* */
// Calculate Adjoint map
// Ad_pose is 6*6 matrix that when applied to twist xi, returns Ad_pose(xi)
Matrix6 Pose3::AdjointMap() const {
  const Matrix3 R = R_.matrix();
  Matrix3 A = skewSymmetric(t_.x(), t_.y(), t_.z()) * R;
  Matrix6 adj;
  adj << R, Z_3x3, A, R;  // Gives [R 0; A R]
  return adj;
}

/* ************************************************************************* */
// Calculate AdjointMap applied to xi_b, with Jacobians
Vector6 Pose3::Adjoint(const Vector6& xi_b, OptionalJacobian<6, 6> H_pose,
                       OptionalJacobian<6, 6> H_xib) const {
  const Matrix6 Ad = AdjointMap();

  // Jacobians
  // D1 Ad_T(xi_b) = D1 Ad_T Ad_I(xi_b) = Ad_T * D1 Ad_I(xi_b) = Ad_T * ad_xi_b
  // D2 Ad_T(xi_b) = Ad_T
  // See docs/math.pdf for more details.
  // In D1 calculation, we could be more efficient by writing it out, but do not
  // for readability
  if (H_pose) *H_pose = -Ad * adjointMap(xi_b);
  if (H_xib) *H_xib = Ad;

  return Ad * xi_b;
}

/* ************************************************************************* */
/// The dual version of Adjoint
Vector6 Pose3::AdjointTranspose(const Vector6& x, OptionalJacobian<6, 6> H_pose,
                                OptionalJacobian<6, 6> H_x) const {
  const Matrix6 Ad = AdjointMap();
  const Vector6 AdTx = Ad.transpose() * x;

  // Jacobians
  // See docs/math.pdf for more details.
  if (H_pose) {
    const auto w_T_hat = skewSymmetric(AdTx.head<3>()),
               v_T_hat = skewSymmetric(AdTx.tail<3>());
    *H_pose << w_T_hat, v_T_hat,  //
        /*  */ v_T_hat, Z_3x3;
  }
  if (H_x) {
    *H_x = Ad.transpose();
  }

  return AdTx;
}

/* ************************************************************************* */
Matrix6 Pose3::adjointMap(const Vector6& xi) {
  Matrix3 w_hat = skewSymmetric(xi(0), xi(1), xi(2));
  Matrix3 v_hat = skewSymmetric(xi(3), xi(4), xi(5));
  Matrix6 adj;
  adj << w_hat, Z_3x3, v_hat, w_hat;

  return adj;
}

/* ************************************************************************* */
Vector6 Pose3::adjoint(const Vector6& xi, const Vector6& y,
                       OptionalJacobian<6, 6> Hxi, OptionalJacobian<6, 6> H_y) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 6; ++i) {
      Vector6 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix6 Gi = adjointMap(dxi);
      Hxi->col(i) = Gi * y;
    }
  }
  const Matrix6& ad_xi = adjointMap(xi);
  if (H_y) *H_y = ad_xi;
  return ad_xi * y;
}

/* ************************************************************************* */
Vector6 Pose3::adjointTranspose(const Vector6& xi, const Vector6& y,
    OptionalJacobian<6, 6> Hxi, OptionalJacobian<6, 6> H_y) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 6; ++i) {
      Vector6 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix6 GTi = adjointMap(dxi).transpose();
      Hxi->col(i) = GTi * y;
    }
  }
  const Matrix6& adT_xi = adjointMap(xi).transpose();
  if (H_y) *H_y = adT_xi;
  return adT_xi * y;
}

/* ************************************************************************* */
Matrix4 Pose3::Hat(const Vector6& xi) {
  Matrix4 X;
  const double wx = xi(0), wy = xi(1), wz = xi(2), vx = xi(3), vy = xi(4), vz = xi(5);
  X << 0., -wz, wy, vx, wz, 0., -wx, vy, -wy, wx, 0., vz, 0., 0., 0., 0.;
  return X;
}

/* ************************************************************************* */
Vector6 Pose3::Vee(const Matrix4& Xi) {
  Vector6 xi;
  xi << Xi(2, 1), Xi(0, 2), Xi(1, 0), Xi(0, 3), Xi(1, 3), Xi(2, 3);
  return xi;
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
    // We are creating a Pose3, so we still need to chain H with R^T, the
    // Jacobian of Pose3::Create with respect to t.
    const Matrix3 Rt = R.transpose();
    *Hxi << Jr, Z_3x3,  // Jr here *is* the Jacobian of expmap
        Rt * H, Jr;     // Here Jr = R^T * Jl, with Jl the Jacobian of t in v.
  }

  return Pose3(R, t);
}

/* ************************************************************************* */
Vector6 Pose3::Logmap(const Pose3& pose, OptionalJacobian<6, 6> Hpose) {
  const Vector3 w = Rot3::Logmap(pose.rotation());

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  const Vector3 t = pose.translation();
  const Vector3 u = local.InvJacobian().applyLeft(t);
  Vector6 xi;
  xi << w, u;
  if (Hpose) *Hpose = LogmapDerivative(xi);
  return xi;
}

/* ************************************************************************* */
Pose3 Pose3::ChartAtOrigin::Retract(const Vector6& xi, ChartJacobian Hxi) {
#ifdef GTSAM_POSE3_EXPMAP
  return Expmap(xi, Hxi);
#else
  Matrix3 DR;
  Rot3 R = Rot3::Retract(xi.head<3>(), Hxi ? &DR : 0);
  if (Hxi) {
    *Hxi = I_6x6;
    Hxi->topLeftCorner<3, 3>() = DR;
  }
  return Pose3(R, Point3(xi.tail<3>()));
#endif
}

/* ************************************************************************* */
Vector6 Pose3::ChartAtOrigin::Local(const Pose3& pose, ChartJacobian Hpose) {
#ifdef GTSAM_POSE3_EXPMAP
  return Logmap(pose, Hpose);
#else
  Matrix3 DR;
  Vector3 omega = Rot3::LocalCoordinates(pose.rotation(), Hpose ? &DR : 0);
  if (Hpose) {
    *Hpose = I_6x6;
    Hpose->topLeftCorner<3, 3>() = DR;
  }
  Vector6 xi;
  xi << omega, pose.translation();
  return xi;
#endif
}

/* ************************************************************************* */
Matrix6 Pose3::ExpmapDerivative(const Vector6& xi) {
  Matrix6 J;
  Expmap(xi, J);
  return J;
}

/* ************************************************************************* */
Matrix6 Pose3::LogmapDerivative(const Vector6& xi) {
  const Vector3 w = xi.head<3>();
  Vector3 v = xi.segment<3>(3);

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  // Call applyLeftJacobian to get its Jacobians
  Matrix3 H_t_w;
  local.Jacobian().applyLeft(v, H_t_w);

  // Multiply with R^T to account for NavState::Create Jacobian.
  const Matrix3 R = local.expmap();
  const Matrix3 Qt = R.transpose() * H_t_w;

  // Now compute the blocks of the LogmapDerivative Jacobian
  const Matrix3 Jw = Rot3::LogmapDerivative(w);
  const Matrix3 Qt2 = -Jw * Qt * Jw;

  Matrix6 J;
  J << Jw, Z_3x3, Qt2, Jw;
  return J;
}

/* ************************************************************************* */
Matrix6 Pose3::LogmapDerivative(const Pose3& pose) {
  const Vector6 xi = Logmap(pose);
  return LogmapDerivative(xi);
}

/* ************************************************************************* */
const Point3& Pose3::translation(OptionalJacobian<3, 6> Hself) const {
  if (Hself) *Hself << Z_3x3, rotation().matrix();
  return t_;
}

/* ************************************************************************* */
const Rot3& Pose3::rotation(OptionalJacobian<3, 6> Hself) const {
  if (Hself) {
    *Hself << I_3x3, Z_3x3;
  }
  return R_;
}

/* ************************************************************************* */
Matrix4 Pose3::matrix() const {
  static const auto A14 = Eigen::RowVector4d(0,0,0,1);
  Matrix4 mat;
  mat << R_.matrix(), t_, A14;
  return mat;
}

/* ************************************************************************* */
Pose3::Vector16 Pose3::vec(OptionalJacobian<16, 6> H) const {
  // Vectorize
  const Matrix4 M = matrix();
  const Vector16 v = Eigen::Map<const Vector16>(M.data());

  // If requested, calculate H
  if (H) {
    H->setZero();
    auto R = M.block<3, 3>(0, 0);
    H->block<3, 1>(0, 1) = -R.col(2);
    H->block<3, 1>(0, 2) = R.col(1);
    H->block<3, 1>(4, 0) = R.col(2);
    H->block<3, 1>(4, 2) = -R.col(0);
    H->block<3, 1>(8, 0) = -R.col(1);
    H->block<3, 1>(8, 1) = R.col(0);
    H->block<3,3>(12,3) = R;
  }

  return v;
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
  Matrix36 D_local_pose;
  Matrix3 D_local_point;
  Point3 local = transformTo(point, Hself ? &D_local_pose : 0, Hpoint ? &D_local_point : 0);
  if (!Hself && !Hpoint) {
    return local.norm();
  } else {
    Matrix13 D_r_local;
    const double r = norm3(local, D_r_local);
    if (Hself) *Hself = D_r_local * D_local_pose;
    if (Hpoint) *Hpoint = D_r_local * D_local_point;
    return r;
  }
}

/* ************************************************************************* */
double Pose3::range(const Pose3& pose, OptionalJacobian<1, 6> Hself,
                    OptionalJacobian<1, 6> Hpose) const {
  Matrix36 D_point_pose;
  Matrix13 D_local_point;
  Point3 point = pose.translation(Hpose ? &D_point_pose : 0);
  double r = range(point, Hself, Hpose ? &D_local_point : 0);
  if (Hpose) *Hpose = D_local_point * D_point_pose;
  return r;
}

/* ************************************************************************* */
Unit3 Pose3::bearing(const Point3& point, OptionalJacobian<2, 6> Hself,
                     OptionalJacobian<2, 3> Hpoint) const {
  Matrix36 D_local_pose;
  Matrix3 D_local_point;
  Point3 at = transformTo(point, Hself ? &D_local_pose : 0, Hpoint ? &D_local_point : 0);
  if (!Hself && !Hpoint) {
    return Unit3(at);
  } else {
    Matrix23 D_b_local;
    Unit3 b = Unit3::FromPoint3(at, D_b_local);
    if (Hself) *Hself = D_b_local * D_local_pose;
    if (Hpoint) *Hpoint = D_b_local * D_local_point;
    return b;
  }
}

/* ************************************************************************* */
Unit3 Pose3::bearing(const Pose3& pose, OptionalJacobian<2, 6> Hself,
  OptionalJacobian<2, 6> Hpose) const {
  Matrix36 D_point_pose;
  Matrix23 D_local_point;
  Point3 point = pose.translation(Hpose ? &D_point_pose : 0);
  Unit3 b = bearing(point, Hself, Hpose ? &D_local_point : 0);
  if (Hpose) *Hpose = D_local_point * D_point_pose;
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
