/**
 * @file  Pose2.cpp
 * @brief 2D Pose
 */

#include "Pose2.h"
#include "Lie-inl.h"

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  template class Lie<Pose2>;

  /* ************************************************************************* */
  void Pose2::print(const string& s) const {
    cout << s << "(" << t_.x() << ", " << t_.y() << ", " << r_.theta() << ")" << endl;
  }

  /* ************************************************************************* */
  bool Pose2::equals(const Pose2& q, double tol) const {
    return t_.equals(q.t_, tol) && r_.equals(q.r_, tol);
  }

  /* ************************************************************************* */
  // TODO, have a combined function that returns both function value and derivative
  Matrix Dtransform_to1(const Pose2& pose, const Point2& point) {
    Matrix dx_dt = Matrix_(2,2, -1.0, 0.0, 0.0, -1.0);
    Matrix dx_dr = Dunrotate1(pose.r(), point-pose.t());
    return collect(2, &dx_dt, &dx_dr);
  }

  Matrix Dtransform_to2(const Pose2& pose, const Point2& point) {
    return pose.r().transpose();
  }

  /* ************************************************************************* */
  Matrix Dbetween1(const Pose2& p0, const Pose2& p2) {
    Matrix dt_dr = Dunrotate1(p0.r(), p2.t()-p0.t());
    Matrix dt_dt1 = -invcompose(p2.r(), p0.r()).matrix();
    Matrix dt_dr1 = Dunrotate1(p2.r(), p2.t()-p0.t());
    return Matrix_(3,3,
        dt_dt1(0,0), dt_dt1(0,1), dt_dr1(0,0),
        dt_dt1(1,0), dt_dt1(1,1), dt_dr1(1,0),
        0.0,         0.0,         -1.0);
  }

  Matrix Dbetween2(const Pose2& p0, const Pose2& p2) {
    Matrix db_dt2 = p0.r().transpose();
    return Matrix_(3,3,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0);
  }

  /* ************************************************************************* */
} // namespace gtsam
