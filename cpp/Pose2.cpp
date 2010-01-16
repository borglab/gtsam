/**
 * @file  Pose2.cpp
 * @brief 2D Pose
 */

#include "Pose2.h"
#include "Lie-inl.h"

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Pose2);

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
  Pose2 between(const Pose2& p1, const Pose2& p2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) {
		Rot2 dR = between(p1.r(), p2.r());
		Point2 dt = p2.t() - p1.t();
		Point2 q = unrotate(p1.r(), dt);
		Pose2 dp(dR, q);
		if (H1) {
			Matrix dT1 = -invcompose(p2.r(), p1.r()).matrix();
			Matrix dR1;
			unrotate(p2.r(), dt, dR1); // FD to Richard: I do *not* understand this
			*H1 = Matrix_(3,3,
				dT1(0,0), dT1(0,1), dR1(0,0),
				dT1(1,0), dT1(1,1), dR1(1,0),
				0.0, 0.0, -1.0);
		}
  	static const Matrix I3 = eye(3);
		if (H2) *H2 = I3;
		return dp;
	}

  /* ************************************************************************* */
	Rot2 bearing(const Pose2& pose, const Point2& point) {
		Point2 d = transform_to(pose, point);
		return relativeBearing(d);
	}

	Rot2 bearing(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		if (!H1 && !H2) return bearing(pose, point);
		Point2 d = transform_to(pose, point);
		Matrix D_result_d;
		Rot2 result = relativeBearing(d, D_result_d);
		if (H1) *H1 = D_result_d * Dtransform_to1(pose, point);
		if (H2) *H2 = D_result_d * Dtransform_to2(pose, point);
		return result;
	}

  /* ************************************************************************* */
	double range(const Pose2& pose, const Point2& point) {
		Point2 d = transform_to(pose, point);
		return d.norm();
	}

	double range(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		if (!H1 && !H2) return range(pose, point);
		Point2 d = transform_to(pose, point);
		double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
		Matrix D_result_d = Matrix_(1, 2, x / n, y / n);
		if (H1) *H1 = D_result_d * Dtransform_to1(pose, point);
		if (H2) *H2 = D_result_d * Dtransform_to2(pose, point);
		return n;
	}

  /* ************************************************************************* */
} // namespace gtsam
