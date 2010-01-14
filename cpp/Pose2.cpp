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
  Point2 transform_to(const Pose2& pose, const Point2& point, boost::optional<
			Matrix&> H1, boost::optional<Matrix&> H2) {
		const Rot2& R = pose.r();
		Point2 d = point - pose.t();
		Point2 q = R.unrotate(d);
		if (!H1 && !H2) return q;
		if (H1) *H1 = Matrix_(2, 3,
					-1.0, 0.0,  q.y(),
					0.0, -1.0, -q.x());
		if (H2) *H2 = R.transpose();
		return q;
	}

  Matrix Dtransform_to1(const Pose2& pose, const Point2& point) {
		Matrix H; transform_to(pose, point, H, boost::none); return H;
  }

  Matrix Dtransform_to2(const Pose2& pose, const Point2& point) {
		Matrix H; transform_to(pose, point, boost::none, H); return H;
  }

  /* ************************************************************************* */
  Matrix Dbetween1(const Pose2& p1, const Pose2& p2) {
  	Point2 dt = p2.t()-p1.t();
    Matrix dT1 = -invcompose(p2.r(), p1.r()).matrix();
    Matrix dR1;
    unrotate(p2.r(), dt, dR1);
    return Matrix_(3,3,
        dT1(0,0), dT1(0,1), dR1(0,0),
        dT1(1,0), dT1(1,1), dR1(1,0),
             0.0,      0.0,     -1.0);
  }

  Matrix Dbetween2(const Pose2& p1, const Pose2& p2) {
    return Matrix_(3,3,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0);
  }

  /* ************************************************************************* */
} // namespace gtsam
