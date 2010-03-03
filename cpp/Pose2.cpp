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

	static const Matrix I3 = eye(3), Z12 = zeros(1,2);
  static const Rot2 R_PI_2(Rot2::fromCosSin(0., 1.));

  /* ************************************************************************* */
  Matrix Pose2::matrix() const {
  	Matrix R = r_.matrix();
  	R = stack(2, &R, &Z12);
  	Matrix T = Matrix_(3,1, t_.x(), t_.y(), 1.0);
  	return collect(2, &R, &T);
  }

  /* ************************************************************************* */
  void Pose2::print(const string& s) const {
    cout << s << "(" << t_.x() << ", " << t_.y() << ", " << r_.theta() << ")" << endl;
  }

  /* ************************************************************************* */
  bool Pose2::equals(const Pose2& q, double tol) const {
    return t_.equals(q.t_, tol) && r_.equals(q.r_, tol);
  }

  /* ************************************************************************* */

#ifdef SLOW_BUT_CORRECT_EXPMAP

	template<> Pose2 expmap(const Vector& xi) {
		Point2 v(xi(0),xi(1));
		double w = xi(2);
		if (fabs(w) < 1e-5)
			return Pose2(xi[0], xi[1], xi[2]);
		else {
			Rot2 R(Rot2::fromAngle(w));
			Point2 v_ortho = R_PI_2 * v; // points towards rot center
			Point2 t = (v_ortho - rotate(R,v_ortho)) / w;
			return Pose2(R, t);
		}
	}

  Vector logmap(const Pose2& p) {
  	const Rot2& R = p.r();
  	const Point2& t = p.t();
		double w = R.theta();
		if (fabs(w) < 1e-5)
			return Vector_(3, t.x(), t.y(), w);
		else {
			double c_1 = R.c()-1.0, s = R.s();
			double det = c_1*c_1 + s*s;
			Point2 p = R_PI_2 * (unrotate(R, t) - t);
			Point2 v = (w/det) * p;
			return Vector_(3, v.x(), v.y(), w);
		}
  }

#else

	template<> Pose2 expmap(const Vector& v) {
		return Pose2(v[0], v[1], v[2]);
	}

	Vector logmap(const Pose2& p) {
		return Vector_(3, p.x(), p.y(), p.theta());
	}

#endif

  /* ************************************************************************* */
  // Calculate Adjoint map
  // Ad_pose is 3*3 matrix that when applied to twist xi, returns Ad_pose(xi)
  Matrix AdjointMap(const Pose2& p) {
		const Rot2 R = p.r();
		const Point2 t = p.t();
  	double c = R.c(), s = R.s(), x = t.x(), y = t.y();
		return Matrix_(3,3,
				  c,  -s,   y,
				  s,   c,  -x,
				0.0, 0.0, 1.0
				);
	}

  /* ************************************************************************* */
  Pose2 inverse(const Pose2& pose) {
		const Rot2& R = pose.r();
		const Point2& t = pose.t();
		return Pose2(inverse(R), R.unrotate(Point2(-t.x(), -t.y())));
	}

	Matrix Dinverse(const Pose2& pose) {
		return -AdjointMap(pose);
	}

  /* ************************************************************************* */
  // see doc/math.lyx, SE(2) section
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

  /* ************************************************************************* */
  // see doc/math.lyx, SE(2) section

  Matrix Dcompose1(const Pose2& p1, const Pose2& p2) {
		return AdjointMap(inverse(p2));
  }

  Matrix Dcompose2(const Pose2& p1, const Pose2& p2) {
  	return I3;
  }

  /* ************************************************************************* */
  // see doc/math.lyx, SE(2) section
  Point2 transform_from(const Pose2& pose, const Point2& p,
  		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
  	const Rot2& rot = pose.r();
		const Point2 q = rot * p;
  	if (H1 || H2) {
			const Matrix R = rot.matrix();
			const Matrix Drotate1 = Matrix_(2, 1, -q.y(), q.x());
	  	if (H1) *H1 = collect(2, &R, &Drotate1); // [R R_{pi/2}q]
			if (H2) *H2 = R;                         // R
  	}
		return q + pose.t();
  }

  /* ************************************************************************* */
  Pose2 between(const Pose2& p1, const Pose2& p2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) {
  	// get cosines and sines from rotation matrices
  	const Rot2& R1 = p1.r(), R2 = p2.r();
  	double c1=R1.c(), s1=R1.s(), c2=R2.c(), s2=R2.s();

  	// Calculate delta rotation = between(R1,R2)
		double c = c1 * c2 + s1 * s2, s = -s1 * c2 + c1 * s2;
    Rot2 R(Rot2::atan2(s,c)); // normalizes

  	// Calculate delta translation = unrotate(R1, dt);
		Point2 dt = p2.t() - p1.t();
		double x = dt.x(), y = dt.y();
		Point2 t(c1 * x + s1 * y, -s1 * x + c1 * y);

		// FD: This is just -AdjointMap(between(p2,p1)) inlined and re-using above
		if (H1) {
			double dt1 = -s2 * x + c2 * y;
			double dt2 = -c2 * x - s2 * y;
			H1->resize(3,3);
			double data[9] = {
				-c,  -s,  dt1,
				 s,  -c,  dt2,
			 0.0, 0.0, -1.0};
			 copy(data, data+9, H1->data().begin());
		}
		if (H2) *H2 = I3;

		return Pose2(R,t);
	}

  /* ************************************************************************* */
	Rot2 bearing(const Pose2& pose, const Point2& point) {
		Point2 d = transform_to(pose, point);
		return relativeBearing(d);
	}

	Rot2 bearing(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
		if (!H1 && !H2) return bearing(pose, point);
		Point2 d = transform_to(pose, point, H1, H2);
		Matrix D_result_d;
		Rot2 result = relativeBearing(d, D_result_d);
		if (H1) *H1 = D_result_d * (*H1);
		if (H2) *H2 = D_result_d * (*H2);
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
		Point2 d = transform_to(pose, point, H1, H2);
		double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
		Matrix D_result_d = Matrix_(1, 2, x / n, y / n);
		if (H1) *H1 = D_result_d * (*H1);
		if (H2) *H2 = D_result_d * (*H2);
		return n;
	}

  /* ************************************************************************* */
} // namespace gtsam
