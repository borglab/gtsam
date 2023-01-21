/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Pose2.cpp
 * @brief 2D Pose
 */

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/concepts.h>

#include <cmath>
#include <iostream>
#include <iomanip>

using namespace std;

namespace gtsam {

/** instantiate concept checks */
GTSAM_CONCEPT_POSE_INST(Pose2)

static const Rot2 R_PI_2(Rot2::fromCosSin(0., 1.));

/* ************************************************************************* */
Matrix3 Pose2::matrix() const {
  Matrix2 R = r_.matrix();
  Matrix32 R0;
  R0.block<2,2>(0,0) = R;
  R0.block<1,2>(2,0).setZero();
  Matrix31 T;
  T <<  t_.x(), t_.y(), 1.0;
  Matrix3 RT_;
  RT_.block<3,2>(0,0) = R0;
  RT_.block<3,1>(0,2) = T;
  return RT_;
}

/* ************************************************************************* */
void Pose2::print(const string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

/* ************************************************************************* */
std::ostream &operator<<(std::ostream &os, const Pose2& pose) {
  os << "(" << pose.x() << ", " << pose.y() << ", " << pose.theta() << ")";
  return os;
}

/* ************************************************************************* */
bool Pose2::equals(const Pose2& q, double tol) const {
  return equal_with_abs_tol(t_, q.t_, tol) && r_.equals(q.r_, tol);
}

/* ************************************************************************* */
Pose2 Pose2::Expmap(const Vector3& xi, OptionalJacobian<3, 3> H) {
  assert(xi.size() == 3);
  if (H) *H = Pose2::ExpmapDerivative(xi);
  const Point2 v(xi(0),xi(1));
  const double w = xi(2);
  if (std::abs(w) < 1e-10)
    return Pose2(xi[0], xi[1], xi[2]);
  else {
    const Rot2 R(Rot2::fromAngle(w));
    const Point2 v_ortho = R_PI_2 * v; // points towards rot center
    const Point2 t = (v_ortho - R.rotate(v_ortho)) / w;
    return Pose2(R, t);
  }
}

/* ************************************************************************* */
Vector3 Pose2::Logmap(const Pose2& p, OptionalJacobian<3, 3> H) {
  if (H) *H = Pose2::LogmapDerivative(p);
  const Rot2& R = p.r();
  const Point2& t = p.t();
  double w = R.theta();
  if (std::abs(w) < 1e-10)
    return Vector3(t.x(), t.y(), w);
  else {
    double c_1 = R.c()-1.0, s = R.s();
    double det = c_1*c_1 + s*s;
    Point2 p = R_PI_2 * (R.unrotate(t) - t);
    Point2 v = (w/det) * p;
    return Vector3(v.x(), v.y(), w);
  }
}

/* ************************************************************************* */
Pose2 Pose2::ChartAtOrigin::Retract(const Vector3& v, ChartJacobian H) {
#ifdef SLOW_BUT_CORRECT_EXPMAP
  return Expmap(v, H);
#else
  if (H) {
    *H = I_3x3;
    H->topLeftCorner<2,2>() = Rot2(-v[2]).matrix();
  }
  return Pose2(v[0], v[1], v[2]);
#endif
}
/* ************************************************************************* */
Vector3 Pose2::ChartAtOrigin::Local(const Pose2& r, ChartJacobian H) {
#ifdef SLOW_BUT_CORRECT_EXPMAP
  return Logmap(r, H);
#else
  if (H) {
    *H = I_3x3;
    H->topLeftCorner<2,2>() = r.rotation().matrix();
  }
  return Vector3(r.x(), r.y(), r.theta());
#endif
}

/* ************************************************************************* */
// Calculate Adjoint map
// Ad_pose is 3*3 matrix that when applied to twist xi, returns Ad_pose(xi)
Matrix3 Pose2::AdjointMap() const {
  double c = r_.c(), s = r_.s(), x = t_.x(), y = t_.y();
  Matrix3 rvalue;
  rvalue <<
      c,  -s,   y,
      s,   c,  -x,
      0.0, 0.0, 1.0;
  return rvalue;
}

/* ************************************************************************* */
Matrix3 Pose2::adjointMap(const Vector3& v) {
  // See Chirikjian12book2, vol.2, pg. 36
  Matrix3 ad = Z_3x3;
  ad(0,1) = -v[2];
  ad(1,0) = v[2];
  ad(0,2) = v[1];
  ad(1,2) = -v[0];
  return ad;
}

/* ************************************************************************* */
Matrix3 Pose2::ExpmapDerivative(const Vector3& v) {
  double alpha = v[2];
  Matrix3 J;
  if (std::abs(alpha) > 1e-5) {
    // Chirikjian11book2, pg. 36
    /* !!!Warning!!! Compare Iserles05an, formula 2.42 and Chirikjian11book2 pg.26
     * Iserles' right-trivialization dexpR is actually the left Jacobian J_l in Chirikjian's notation
     * In fact, Iserles 2.42 can be written as:
     *    \dot{g} g^{-1} = dexpR_{q}\dot{q}
     * where q = A, and g = exp(A)
     * and the LHS is in the definition of J_l in Chirikjian11book2, pg. 26.
     * Hence, to compute ExpmapDerivative, we have to use the formula of J_r Chirikjian11book2, pg.36
     */
    double sZalpha = sin(alpha)/alpha, c_1Zalpha = (cos(alpha)-1)/alpha;
    double v1Zalpha = v[0]/alpha, v2Zalpha = v[1]/alpha;
    J << sZalpha, -c_1Zalpha, v1Zalpha + v2Zalpha*c_1Zalpha - v1Zalpha*sZalpha,
         c_1Zalpha, sZalpha, -v1Zalpha*c_1Zalpha + v2Zalpha - v2Zalpha*sZalpha,
         0, 0, 1;
  }
  else {
    // Thanks to Krunal: Apply L'Hospital rule to several times to
    // compute the limits when alpha -> 0
    J << 1,0,-0.5*v[1],
        0,1, 0.5*v[0],
        0,0, 1;
  }

  return J;
}

/* ************************************************************************* */
Matrix3 Pose2::LogmapDerivative(const Pose2& p) {
  Vector3 v = Logmap(p);
  double alpha = v[2];
  Matrix3 J;
  if (std::abs(alpha) > 1e-5) {
    double alphaInv = 1/alpha;
    double halfCotHalfAlpha = 0.5*sin(alpha)/(1-cos(alpha));
    double v1 = v[0], v2 = v[1];

    J << alpha*halfCotHalfAlpha, -0.5*alpha, v1*alphaInv - v1*halfCotHalfAlpha + 0.5*v2,
         0.5*alpha, alpha*halfCotHalfAlpha,  v2*alphaInv - 0.5*v1 - v2*halfCotHalfAlpha,
         0, 0, 1;
  }
  else {
    J << 1,0, 0.5*v[1],
        0,1, -0.5*v[0],
        0,0, 1;
  }
  return J;
}

/* ************************************************************************* */
Pose2 Pose2::inverse() const {
  return Pose2(r_.inverse(), r_.unrotate(Point2(-t_.x(), -t_.y())));
}

/* ************************************************************************* */
// see doc/math.lyx, SE(2) section
Point2 Pose2::transformTo(const Point2& point,
    OptionalJacobian<2, 3> Hpose, OptionalJacobian<2, 2> Hpoint) const {
  OptionalJacobian<2, 2> Htranslation = Hpose.cols<2>(0);
  OptionalJacobian<2, 1> Hrotation = Hpose.cols<1>(2);
  const Point2 q = r_.unrotate(point - t_, Hrotation, Hpoint);
  if (Htranslation) *Htranslation << -1.0, 0.0, 0.0, -1.0;
  return q;
}

Matrix Pose2::transformTo(const Matrix& points) const {
  if (points.rows() != 2) {
    throw std::invalid_argument("Pose2:transformTo expects 2*N matrix.");
  }
  const Matrix2 Rt = rotation().transpose();
  return Rt * (points.colwise() - t_);  // Eigen broadcasting!
}

/* ************************************************************************* */
// see doc/math.lyx, SE(2) section
Point2 Pose2::transformFrom(const Point2& point,
    OptionalJacobian<2, 3> Hpose, OptionalJacobian<2, 2> Hpoint) const {
  OptionalJacobian<2, 2> Htranslation = Hpose.cols<2>(0);
  OptionalJacobian<2, 1> Hrotation = Hpose.cols<1>(2);
  const Point2 q = r_.rotate(point, Hrotation, Hpoint);
  if (Htranslation) *Htranslation = (Hpoint ? *Hpoint : r_.matrix());
  return q + t_;
}


Matrix Pose2::transformFrom(const Matrix& points) const {
  if (points.rows() != 2) {
    throw std::invalid_argument("Pose2:transformFrom expects 2*N matrix.");
  }
  const Matrix2 R = rotation().matrix();
  return (R * points).colwise() + t_;  // Eigen broadcasting!
}

/* ************************************************************************* */
Rot2 Pose2::bearing(const Point2& point,
    OptionalJacobian<1, 3> Hpose, OptionalJacobian<1, 2> Hpoint) const {
  // make temporary matrices
  Matrix23 D_d_pose; Matrix2 D_d_point;
  Point2 d = transformTo(point, Hpose ? &D_d_pose : 0, Hpoint ? &D_d_point : 0);
  if (!Hpose && !Hpoint) return Rot2::relativeBearing(d);
  Matrix12 D_result_d;
  Rot2 result = Rot2::relativeBearing(d, Hpose || Hpoint ? &D_result_d : 0);
  if (Hpose) *Hpose = D_result_d * D_d_pose;
  if (Hpoint) *Hpoint = D_result_d * D_d_point;
  return result;
}

/* ************************************************************************* */
Rot2 Pose2::bearing(const Pose2& pose,
    OptionalJacobian<1, 3> Hpose, OptionalJacobian<1, 3> Hother) const {
  Matrix12 D2;
  Rot2 result = bearing(pose.t(), Hpose, Hother ? &D2 : 0);
  if (Hother) {
    Matrix12 H2_ = D2 * pose.r().matrix();
    *Hother << H2_, Z_1x1;
  }
  return result;
}
/* ************************************************************************* */
double Pose2::range(const Point2& point,
    OptionalJacobian<1,3> Hpose, OptionalJacobian<1,2> Hpoint) const {
  Point2 d = point - t_;
  if (!Hpose && !Hpoint) return d.norm();
  Matrix12 D_r_d;
  double r = norm2(d, D_r_d);
  if (Hpose) {
      Matrix23 D_d_pose;
      D_d_pose << -r_.c(),  r_.s(),  0.0,
                  -r_.s(), -r_.c(),  0.0;
      *Hpose = D_r_d * D_d_pose;
  }
  if (Hpoint) *Hpoint = D_r_d;
  return r;
}

/* ************************************************************************* */
double Pose2::range(const Pose2& pose,
    OptionalJacobian<1,3> Hpose,
    OptionalJacobian<1,3> Hother) const {
  Point2 d = pose.t() - t_;
  if (!Hpose && !Hother) return d.norm();
  Matrix12 D_r_d;
  double r = norm2(d, D_r_d);
  if (Hpose) {
      Matrix23 D_d_pose;
      D_d_pose <<
      -r_.c(),  r_.s(),  0.0,
      -r_.s(), -r_.c(),  0.0;
      *Hpose = D_r_d * D_d_pose;
  }
  if (Hother) {
      Matrix23 D_d_other;
      D_d_other <<
      pose.r_.c(), -pose.r_.s(),  0.0,
      pose.r_.s(),  pose.r_.c(),  0.0;
      *Hother = D_r_d * D_d_other;
  }
  return r;
}

/* *************************************************************************
 * Align finds the angle using a linear method:
 * a = Pose2::transformFrom(b) = t + R*b
 * We need to remove the centroids from the data to find the rotation
 * using db=[dbx;dby] and a=[dax;day] we have
 *  |dax|   |c  -s|     |dbx|     |dbx -dby|     |c|
 *  |   | = |     |  *  |   |  =  |        |  *  | | = H_i*cs
 *  |day|   |s   c|     |dby|     |dby  dbx|     |s|
 * where the Hi are the 2*2 matrices. Then we will minimize the criterion
 * J = \sum_i norm(a_i - H_i * cs)
 * Taking the derivative with respect to cs and setting to zero we have
 * cs = (\sum_i H_i' * a_i)/(\sum H_i'*H_i)
 * The hessian is diagonal and just divides by a constant, but this
 * normalization constant is irrelevant, since we take atan2.
 * i.e., cos ~ sum(dbx*dax + dby*day) and sin ~ sum(-dby*dax + dbx*day)
 * The translation is then found from the centroids
 * as they also satisfy ca = t + R*cb, hence t = ca - R*cb
 */

boost::optional<Pose2> Pose2::Align(const Point2Pairs &ab_pairs) {
  const size_t n = ab_pairs.size();
  if (n < 2) {
    return boost::none;  // we need at least 2 pairs
  }

  // calculate centroids
  Point2 ca(0, 0), cb(0, 0);
  for (const Point2Pair& pair : ab_pairs) {
    ca += pair.first;
    cb += pair.second;
  }
  const double f = 1.0/n;
  ca *= f;
  cb *= f;

  // calculate cos and sin
  double c = 0, s = 0;
  for (const Point2Pair& pair : ab_pairs) {
    Point2 da = pair.first - ca;
    Point2 db = pair.second - cb;
    c += db.x() * da.x() + db.y() * da.y();
    s += -db.y() * da.x() + db.x() * da.y();
  }

  // calculate angle and translation
  const double theta = atan2(s, c);
  const Rot2 R = Rot2::fromAngle(theta);
  const Point2 t = ca - R*cb;
  return Pose2(R, t);
}

boost::optional<Pose2> Pose2::Align(const Matrix& a, const Matrix& b) {
  if (a.rows() != 2 || b.rows() != 2 || a.cols() != b.cols()) {
    throw std::invalid_argument(
      "Pose2:Align expects 2*N matrices of equal shape.");
  }
  Point2Pairs ab_pairs;
  for (Eigen::Index j = 0; j < a.cols(); j++) {
    ab_pairs.emplace_back(a.col(j), b.col(j));
  }
  return Pose2::Align(ab_pairs);
}

/* ************************************************************************* */
} // namespace gtsam
