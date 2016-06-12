/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Pose3.cpp
 * @brief 3D Pose
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/concepts.h>

#include <iostream>
#include <cmath>

using namespace std;

namespace gtsam {

/** instantiate concept checks */
GTSAM_CONCEPT_POSE_INST(Pose3);

/* ************************************************************************* */
Pose3::Pose3(const Pose2& pose2) :
    R_(Rot3::Rodrigues(0, 0, pose2.theta())), t_(
        Point3(pose2.x(), pose2.y(), 0)) {
}

/* ************************************************************************* */
Pose3 Pose3::Create(const Rot3& R, const Point3& t, OptionalJacobian<6, 3> H1,
                    OptionalJacobian<6, 3> H2) {
  if (H1) *H1 << I_3x3, Z_3x3;
  if (H2) *H2 << Z_3x3, R.transpose();
  return Pose3(R, t);
}

/* ************************************************************************* */
Pose3 Pose3::inverse() const {
  Rot3 Rt = R_.inverse();
  return Pose3(Rt, Rt * (-t_));
}

/* ************************************************************************* */
// Calculate Adjoint map
// Ad_pose is 6*6 matrix that when applied to twist xi, returns Ad_pose(xi)
// Experimental - unit tests of derivatives based on it do not check out yet
Matrix6 Pose3::AdjointMap() const {
  const Matrix3 R = R_.matrix();
  Matrix3 A = skewSymmetric(t_.x(), t_.y(), t_.z()) * R;
  Matrix6 adj;
  adj << R, Z_3x3, A, R;
  return adj;
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
    OptionalJacobian<6,6> H) {
  if (H) {
    H->setZero();
    for (int i = 0; i < 6; ++i) {
      Vector6 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix6 Gi = adjointMap(dxi);
      H->col(i) = Gi * y;
    }
  }
  return adjointMap(xi) * y;
}

/* ************************************************************************* */
Vector6 Pose3::adjointTranspose(const Vector6& xi, const Vector6& y,
    OptionalJacobian<6,6> H) {
  if (H) {
    H->setZero();
    for (int i = 0; i < 6; ++i) {
      Vector6 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix6 GTi = adjointMap(dxi).transpose();
      H->col(i) = GTi * y;
    }
  }
  return adjointMap(xi).transpose() * y;
}

/* ************************************************************************* */
void Pose3::print(const string& s) const {
  cout << s;
  R_.print("R:\n");
  cout << '[' << t_.x() << ", " << t_.y() << ", " << t_.z() << "]\';";
}

/* ************************************************************************* */
bool Pose3::equals(const Pose3& pose, double tol) const {
  return R_.equals(pose.R_, tol) && traits<Point3>::Equals(t_, pose.t_, tol);
}

/* ************************************************************************* */
/** Modified from Murray94book version (which assumes w and v normalized?) */
Pose3 Pose3::Expmap(const Vector6& xi, OptionalJacobian<6, 6> H) {
  if (H) *H = ExpmapDerivative(xi);

  // get angular velocity omega and translational velocity v from twist xi
  Vector3 omega(xi(0), xi(1), xi(2)), v(xi(3), xi(4), xi(5));

  Rot3 R = Rot3::Expmap(omega);
  double theta2 = omega.dot(omega);
  if (theta2 > std::numeric_limits<double>::epsilon()) {
    Vector3 t_parallel = omega * omega.dot(v); // translation parallel to axis
    Vector3 omega_cross_v = omega.cross(v);    // points towards axis
    Vector3 t = (omega_cross_v - R * omega_cross_v + t_parallel) / theta2;
    return Pose3(R, t);
  } else {
    return Pose3(R, v);
  }
}

/* ************************************************************************* */
Vector6 Pose3::Logmap(const Pose3& p, OptionalJacobian<6, 6> H) {
  if (H) *H = LogmapDerivative(p);
  const Vector3 w = Rot3::Logmap(p.rotation());
  const Vector3 T = p.translation();
  const double t = w.norm();
  if (t < 1e-10) {
    Vector6 log;
    log << w, T;
    return log;
  } else {
    const Matrix3 W = skewSymmetric(w / t);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in T to avoid matrix math
    const double Tan = tan(0.5 * t);
    const Vector3 WT = W * T;
    const Vector3 u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    Vector6 log;
    log << w, u;
    return log;
  }
}

/* ************************************************************************* */
Pose3 Pose3::ChartAtOrigin::Retract(const Vector6& xi, ChartJacobian H) {
#ifdef GTSAM_POSE3_EXPMAP
  return Expmap(xi, H);
#else
  Matrix3 DR;
  Rot3 R = Rot3::Retract(xi.head<3>(), H ? &DR : 0);
  if (H) {
    *H = I_6x6;
    H->topLeftCorner<3,3>() = DR;
  }
  return Pose3(R, Point3(xi.tail<3>()));
#endif
}

/* ************************************************************************* */
Vector6 Pose3::ChartAtOrigin::Local(const Pose3& T, ChartJacobian H) {
#ifdef GTSAM_POSE3_EXPMAP
  return Logmap(T, H);
#else
  Matrix3 DR;
  Vector3 omega = Rot3::LocalCoordinates(T.rotation(), H ? &DR : 0);
  if (H) {
    *H = I_6x6;
    H->topLeftCorner<3,3>() = DR;
  }
  Vector6 xi;
  xi << omega, T.translation();
  return xi;
#endif
}

/* ************************************************************************* */
/**
 * Compute the 3x3 bottom-left block Q of the SE3 Expmap derivative matrix
 *  J(xi) = [J_(w) Z_3x3;
 *             Q   J_(w)]
 *  where J_(w) is the SO3 Expmap derivative.
 *  (see Chirikjian11book2, pg 44, eq 10.95.
 *  The closed-form formula is similar to formula 102 in Barfoot14tro)
 */
static Matrix3 computeQforExpmapDerivative(const Vector6& xi) {
  const auto w = xi.head<3>();
  const auto v = xi.tail<3>();
  const Matrix3 V = skewSymmetric(v);
  const Matrix3 W = skewSymmetric(w);

  Matrix3 Q;

#ifdef NUMERICAL_EXPMAP_DERIV
  Matrix3 Qj = Z_3x3;
  double invFac = 1.0;
  Q = Z_3x3;
  Matrix3 Wj = I_3x3;
  for (size_t j=1; j<10; ++j) {
    Qj = Qj*W + Wj*V;
    invFac = -invFac/(j+1);
    Q = Q + invFac*Qj;
    Wj = Wj*W;
  }
#else
  // The closed-form formula in Barfoot14tro eq. (102)
  double phi = w.norm();
  if (fabs(phi)>1e-5) {
    const double sinPhi = sin(phi), cosPhi = cos(phi);
    const double phi2 = phi * phi, phi3 = phi2 * phi, phi4 = phi3 * phi, phi5 = phi4 * phi;
    // Invert the sign of odd-order terms to have the right Jacobian
    Q = -0.5*V + (phi-sinPhi)/phi3*(W*V + V*W - W*V*W)
            + (1-phi2/2-cosPhi)/phi4*(W*W*V + V*W*W - 3*W*V*W)
            - 0.5*((1-phi2/2-cosPhi)/phi4 - 3*(phi-sinPhi-phi3/6.)/phi5)*(W*V*W*W + W*W*V*W);
  }
  else {
    Q = -0.5*V + 1./6.*(W*V + V*W - W*V*W)
        + 1./24.*(W*W*V + V*W*W - 3*W*V*W)
        - 0.5*(1./24. + 3./120.)*(W*V*W*W + W*W*V*W);
  }
#endif

  return Q;
}

/* ************************************************************************* */
Matrix6 Pose3::ExpmapDerivative(const Vector6& xi) {
  const Vector3 w = xi.head<3>();
  const Matrix3 Jw = Rot3::ExpmapDerivative(w);
  const Matrix3 Q = computeQforExpmapDerivative(xi);
  Matrix6 J;
  J << Jw, Z_3x3, Q, Jw;
  return J;
}

/* ************************************************************************* */
Matrix6 Pose3::LogmapDerivative(const Pose3& pose) {
  const Vector6 xi = Logmap(pose);
  const Vector3 w = xi.head<3>();
  const Matrix3 Jw = Rot3::LogmapDerivative(w);
  const Matrix3 Q = computeQforExpmapDerivative(xi);
  const Matrix3 Q2 = -Jw*Q*Jw;
  Matrix6 J;
  J << Jw, Z_3x3, Q2, Jw;
  return J;
}

/* ************************************************************************* */
const Point3& Pose3::translation(OptionalJacobian<3, 6> H) const {
  if (H) *H << Z_3x3, rotation().matrix();
  return t_;
}

/* ************************************************************************* */

const Rot3& Pose3::rotation(OptionalJacobian<3, 6> H) const {
  if (H) {
    *H << I_3x3, Z_3x3;
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
Pose3 Pose3::transform_to(const Pose3& pose) const {
  Rot3 cRv = R_ * Rot3(pose.R_.inverse());
  Point3 t = pose.transform_to(t_);
  return Pose3(cRv, t);
}

/* ************************************************************************* */
Pose3 Pose3::transform_pose_to(const Pose3& pose, OptionalJacobian<6, 6> H1,
                                                  OptionalJacobian<6, 6> H2) const {
  if (H1) *H1 = -pose.inverse().AdjointMap() * AdjointMap();
  if (H2) *H2 = I_6x6;
  return inverse() * pose;
}

/* ************************************************************************* */
Point3 Pose3::transform_from(const Point3& p, OptionalJacobian<3,6> Dpose,
    OptionalJacobian<3,3> Dpoint) const {
  // Only get matrix once, to avoid multiple allocations,
  // as well as multiple conversions in the Quaternion case
  const Matrix3 R = R_.matrix();
  if (Dpose) {
    Dpose->leftCols<3>() = R * skewSymmetric(-p.x(), -p.y(), -p.z());
    Dpose->rightCols<3>() = R;
  }
  if (Dpoint) {
    *Dpoint = R;
  }
  return R_ * p + t_;
}

/* ************************************************************************* */
Point3 Pose3::transform_to(const Point3& p, OptionalJacobian<3,6> Dpose,
    OptionalJacobian<3,3> Dpoint) const {
  // Only get transpose once, to avoid multiple allocations,
  // as well as multiple conversions in the Quaternion case
  const Matrix3 Rt = R_.transpose();
  const Point3 q(Rt*(p - t_));
  if (Dpose) {
    const double wx = q.x(), wy = q.y(), wz = q.z();
    (*Dpose) <<
        0.0, -wz, +wy,-1.0, 0.0, 0.0,
        +wz, 0.0, -wx, 0.0,-1.0, 0.0,
        -wy, +wx, 0.0, 0.0, 0.0,-1.0;
  }
  if (Dpoint) {
    *Dpoint = Rt;
  }
  return q;
}

/* ************************************************************************* */
double Pose3::range(const Point3& point, OptionalJacobian<1, 6> H1,
                    OptionalJacobian<1, 3> H2) const {
  Matrix36 D_local_pose;
  Matrix3 D_local_point;
  Point3 local = transform_to(point, H1 ? &D_local_pose : 0, H2 ? &D_local_point : 0);
  if (!H1 && !H2) {
    return local.norm();
  } else {
    Matrix13 D_r_local;
    const double r = norm3(local, D_r_local);
    if (H1) *H1 = D_r_local * D_local_pose;
    if (H2) *H2 = D_r_local * D_local_point;
    return r;
  }
}

/* ************************************************************************* */
double Pose3::range(const Pose3& pose, OptionalJacobian<1, 6> H1,
                    OptionalJacobian<1, 6> H2) const {
  Matrix13 D_local_point;
  double r = range(pose.translation(), H1, H2 ? &D_local_point : 0);
  if (H2) *H2 << Matrix13::Zero(), D_local_point * pose.rotation().matrix();
  return r;
}

/* ************************************************************************* */
Unit3 Pose3::bearing(const Point3& point, OptionalJacobian<2, 6> H1,
                     OptionalJacobian<2, 3> H2) const {
  Matrix36 D_local_pose;
  Matrix3 D_local_point;
  Point3 local = transform_to(point, H1 ? &D_local_pose : 0, H2 ? &D_local_point : 0);
  if (!H1 && !H2) {
    return Unit3(local);
  } else {
    Matrix23 D_b_local;
    Unit3 b = Unit3::FromPoint3(local, D_b_local);
    if (H1) *H1 = D_b_local * D_local_pose;
    if (H2) *H2 = D_b_local * D_local_point;
    return b;
  }
}

/* ************************************************************************* */
boost::optional<Pose3> Pose3::Align(const std::vector<Point3Pair>& abPointPairs) {
  const size_t n = abPointPairs.size();
  if (n < 3)
    return boost::none;  // we need at least three pairs

  // calculate centroids
  Point3 aCentroid(0,0,0), bCentroid(0,0,0);
  for(const Point3Pair& abPair: abPointPairs) {
    aCentroid += abPair.first;
    bCentroid += abPair.second;
  }
  double f = 1.0 / n;
  aCentroid *= f;
  bCentroid *= f;

  // Add to form H matrix
  Matrix3 H = Z_3x3;
  for(const Point3Pair& abPair: abPointPairs) {
    Point3 da = abPair.first - aCentroid;
    Point3 db = abPair.second - bCentroid;
    H += db * da.transpose();
    }

  // Compute SVD
  Eigen::JacobiSVD<Matrix> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Matrix U = svd.matrixU();
  Vector S = svd.singularValues();
  Matrix V = svd.matrixV();

  // Check rank
  if (S[1] < 1e-10)
    return boost::none;

  // Recover transform with correction from Eggert97machinevisionandapplications
  Matrix3 UVtranspose = U * V.transpose();
  Matrix3 detWeighting = I_3x3;
  detWeighting(2, 2) = UVtranspose.determinant();
  Rot3 aRb(Matrix(V * detWeighting * U.transpose()));
  Point3 aTb = Point3(aCentroid) - aRb * Point3(bCentroid);
  return Pose3(aRb, aTb);
}

boost::optional<Pose3> align(const vector<Point3Pair>& baPointPairs) {
  vector<Point3Pair> abPointPairs;
  for (const Point3Pair& baPair: baPointPairs) {
    abPointPairs.push_back(make_pair(baPair.second, baPair.first));
  }
  return Pose3::Align(abPointPairs);
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Pose3& pose) {
  os << pose.rotation() << "\n";
  const Point3& t = pose.translation();
  os << '[' << t.x() << ", " << t.y() << ", " << t.z() << "]\';\n";
  return os;
}

} // namespace gtsam
