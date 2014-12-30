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

#include <boost/foreach.hpp>
#include <iostream>
#include <cmath>

using namespace std;

namespace gtsam {

/** instantiate concept checks */
GTSAM_CONCEPT_POSE_INST(Pose3);

/* ************************************************************************* */
Pose3::Pose3(const Pose2& pose2) :
    R_(Rot3::rodriguez(0, 0, pose2.theta())), t_(
        Point3(pose2.x(), pose2.y(), 0)) {
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
  const Vector3 t = t_.vector();
  Matrix3 A = skewSymmetric(t) * R;
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
  t_.print("t: ");
}

/* ************************************************************************* */
bool Pose3::equals(const Pose3& pose, double tol) const {
  return R_.equals(pose.R_, tol) && t_.equals(pose.t_, tol);
}

/* ************************************************************************* */
/** Modified from Murray94book version (which assumes w and v normalized?) */
Pose3 Pose3::Expmap(const Vector& xi, OptionalJacobian<6, 6> H) {
  if (H) *H = ExpmapDerivative(xi);

  // get angular velocity omega and translational velocity v from twist xi
  Point3 w(xi(0), xi(1), xi(2)), v(xi(3), xi(4), xi(5));

  double theta = w.norm();
  if (theta < 1e-10) {
    static const Rot3 I;
    return Pose3(I, v);
  } else {
    Point3 n(w / theta); // axis unit vector
    Rot3 R = Rot3::rodriguez(n.vector(), theta);
    double vn = n.dot(v); // translation parallel to n
    Point3 n_cross_v = n.cross(v); // points towards axis
    Point3 t = (n_cross_v - R * n_cross_v) / theta + vn * n;
    return Pose3(R, t);
  }
}

/* ************************************************************************* */
Vector6 Pose3::Logmap(const Pose3& p, OptionalJacobian<6, 6> H) {
  if (H) *H = LogmapDerivative(p);
  Vector3 w = Rot3::Logmap(p.rotation()), T = p.translation().vector();
  double t = w.norm();
  if (t < 1e-10) {
    Vector6 log;
    log << w, T;
    return log;
  } else {
    Matrix3 W = skewSymmetric(w / t);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in T to avoid matrix math
    double Tan = tan(0.5 * t);
    Vector3 WT = W * T;
    Vector3 u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
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
  xi << omega, T.translation().vector();
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
  Vector3 w(sub(xi, 0, 3));
  Vector3 v(sub(xi, 3, 6));
  Matrix3 V = skewSymmetric(v);
  Matrix3 W = skewSymmetric(w);

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
    double sinPhi = sin(phi), cosPhi = cos(phi);
    double phi2 = phi * phi, phi3 = phi2 * phi, phi4 = phi3 * phi, phi5 = phi4 * phi;
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
  Vector3 w(sub(xi, 0, 3));
  Matrix3 Jw = Rot3::ExpmapDerivative(w);
  Matrix3 Q = computeQforExpmapDerivative(xi);
  Matrix6 J = (Matrix(6,6) << Jw, Z_3x3, Q, Jw).finished();
  return J;
}

/* ************************************************************************* */
Matrix6 Pose3::LogmapDerivative(const Pose3& pose) {
  Vector6 xi = Logmap(pose);
  Matrix3 Jw = Rot3::LogmapDerivative(pose.rotation());
  Matrix3 Q = computeQforExpmapDerivative(xi);
  Matrix3 Q2 = -Jw*Q*Jw;
  Matrix6 J = (Matrix(6,6) << Jw, Z_3x3, Q2, Jw).finished();
  return J;
}

/* ************************************************************************* */
Matrix4 Pose3::matrix() const {
  const Matrix3 R = R_.matrix();
  const Vector3 T = t_.vector();
  Matrix14 A14;
  A14 << 0.0, 0.0, 0.0, 1.0;
  Matrix4 mat;
  mat << R, T, A14;
  return mat;
}

/* ************************************************************************* */
Pose3 Pose3::transform_to(const Pose3& pose) const {
  Rot3 cRv = R_ * Rot3(pose.R_.inverse());
  Point3 t = pose.transform_to(t_);
  return Pose3(cRv, t);
}

/* ************************************************************************* */
Point3 Pose3::transform_from(const Point3& p, OptionalJacobian<3,6> Dpose,
    OptionalJacobian<3,3> Dpoint) const {
  if (Dpose) {
    const Matrix3 R = R_.matrix();
    Matrix3 DR = R * skewSymmetric(-p.x(), -p.y(), -p.z());
    (*Dpose) << DR, R;
  }
  if (Dpoint)
    *Dpoint = R_.matrix();
  return R_ * p + t_;
}

/* ************************************************************************* */
Point3 Pose3::transform_to(const Point3& p, OptionalJacobian<3,6> Dpose,
    OptionalJacobian<3,3> Dpoint) const {
  // Only get transpose once, to avoid multiple allocations,
  // as well as multiple conversions in the Quaternion case
  const Matrix3 Rt = R_.transpose();
  const Point3 q(Rt*(p - t_).vector());
  if (Dpose) {
    const double wx = q.x(), wy = q.y(), wz = q.z();
    (*Dpose) <<
        0.0, -wz, +wy,-1.0, 0.0, 0.0,
        +wz, 0.0, -wx, 0.0,-1.0, 0.0,
        -wy, +wx, 0.0, 0.0, 0.0,-1.0;
  }
  if (Dpoint)
    *Dpoint = Rt;
  return q;
}

/* ************************************************************************* */
double Pose3::range(const Point3& point, OptionalJacobian<1, 6> H1,
    OptionalJacobian<1, 3> H2) const {
  if (!H1 && !H2)
    return transform_to(point).norm();
  else {
    Matrix36 D1;
    Matrix3 D2;
    Point3 d = transform_to(point, H1 ? &D1 : 0, H2 ? &D2 : 0);
    const double x = d.x(), y = d.y(), z = d.z(), d2 = x * x + y * y + z * z,
        n = sqrt(d2);
    Matrix13 D_result_d;
    D_result_d << x / n, y / n, z / n;
    if (H1) *H1 = D_result_d * D1;
    if (H2) *H2 = D_result_d * D2;
    return n;
  }
}

/* ************************************************************************* */
double Pose3::range(const Pose3& pose, OptionalJacobian<1,6> H1,
    OptionalJacobian<1,6> H2) const {
  Matrix13 D2;
  double r = range(pose.translation(), H1, H2? &D2 : 0);
  if (H2) {
    Matrix13 H2_ = D2 * pose.rotation().matrix();
    *H2 << Matrix13::Zero(), H2_;
  }
  return r;
}

/* ************************************************************************* */
boost::optional<Pose3> align(const vector<Point3Pair>& pairs) {
  const size_t n = pairs.size();
  if (n < 3)
    return boost::none; // we need at least three pairs

  // calculate centroids
  Vector cp = zero(3), cq = zero(3);
  BOOST_FOREACH(const Point3Pair& pair, pairs){
  cp += pair.first.vector();
  cq += pair.second.vector();
}
  double f = 1.0 / n;
  cp *= f;
  cq *= f;

  // Add to form H matrix
  Matrix3 H = Eigen::Matrix3d::Zero();
  BOOST_FOREACH(const Point3Pair& pair, pairs){
  Vector dp = pair.first.vector() - cp;
  Vector dq = pair.second.vector() - cq;
  H += dp * dq.transpose();
}

// Compute SVD
  Matrix U,V;
  Vector S;
  svd(H, U, S, V);

  // Recover transform with correction from Eggert97machinevisionandapplications
  Matrix3 UVtranspose = U * V.transpose();
  Matrix3 detWeighting = I_3x3;
  detWeighting(2, 2) = UVtranspose.determinant();
  Rot3 R(Matrix(V * detWeighting * U.transpose()));
  Point3 t = Point3(cq) - R * Point3(cp);
  return Pose3(R, t);
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Pose3& pose) {
  os << pose.rotation() << "\n" << pose.translation() << endl;
  return os;
}

} // namespace gtsam
