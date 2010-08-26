/**
 * @file  Pose2.cpp
 * @brief 2D Pose
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Lie-inl.h>

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
			Point2 t = (v_ortho - R.rotate(v_ortho)) / w;
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
			Point2 p = R_PI_2 * (R.unrotate(t) - t);
			Point2 v = (w/det) * p;
			return Vector_(3, v.x(), v.y(), w);
		}
  }

#else

  /* ************************************************************************* */
  Pose2 Pose2::Expmap(const Vector& v) {
	  return Pose2(v[0], v[1], v[2]);
  }

  /* ************************************************************************* */
  Vector Pose2::Logmap(const Pose2& p) {
	  return Vector_(3, p.x(), p.y(), p.theta());
  }

#endif

  /* ************************************************************************* */
  // Calculate Adjoint map
  // Ad_pose is 3*3 matrix that when applied to twist xi, returns Ad_pose(xi)
  Matrix Pose2::AdjointMap() const {
	  double c = r_.c(), s = r_.s(), x = t_.x(), y = t_.y();
	  return Matrix_(3,3,
			  c,  -s,   y,
			  s,   c,  -x,
			  0.0, 0.0, 1.0
	  );
  }

  /* ************************************************************************* */
  Pose2 Pose2::inverse(boost::optional<Matrix&> H1) const {
	  if (H1) *H1 = -AdjointMap();
	  return Pose2(r_.inverse(), r_.unrotate(Point2(-t_.x(), -t_.y())));
  }

  /* ************************************************************************* */
  // see doc/math.lyx, SE(2) section
  Point2 Pose2::transform_to(const Point2& point,
		  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
		Point2 d = point - t_;
		Point2 q = r_.unrotate(d);
		if (!H1 && !H2) return q;
		if (H1) *H1 = Matrix_(2, 3,
					-1.0, 0.0,  q.y(),
					0.0, -1.0, -q.x());
		if (H2) *H2 = r_.transpose();
		return q;
	}

  /* ************************************************************************* */
  // see doc/math.lyx, SE(2) section
  Pose2 Pose2::compose(const Pose2& p2, boost::optional<Matrix&> H1,
      boost::optional<Matrix&> H2) const {
    // TODO: inline and reuse?
    if(H1) *H1 = p2.inverse().AdjointMap();
    if(H2) *H2 = I3;
    return (*this)*p2;
  }

  /* ************************************************************************* */
  // see doc/math.lyx, SE(2) section
  Point2 Pose2::transform_from(const Point2& p,
		  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  const Point2 q = r_ * p;
	  if (H1 || H2) {
		  const Matrix R = r_.matrix();
		  const Matrix Drotate1 = Matrix_(2, 1, -q.y(), q.x());
		  if (H1) *H1 = collect(2, &R, &Drotate1); // [R R_{pi/2}q]
		  if (H2) *H2 = R;                         // R
	  }
	  return q + t_;
  }

  /* ************************************************************************* */
  Pose2 Pose2::between(const Pose2& p2, boost::optional<Matrix&> H1,
		  boost::optional<Matrix&> H2) const {
	  // get cosines and sines from rotation matrices
	  const Rot2& R1 = r_, R2 = p2.r();
	  double c1=R1.c(), s1=R1.s(), c2=R2.c(), s2=R2.s();

	  // Calculate delta rotation = between(R1,R2)
	  double c = c1 * c2 + s1 * s2, s = -s1 * c2 + c1 * s2;
	  Rot2 R(Rot2::atan2(s,c)); // normalizes

	  // Calculate delta translation = unrotate(R1, dt);
	  Point2 dt = p2.t() - t_;
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
  Rot2 Pose2::bearing(const Point2& point,
		  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  Point2 d = transform_to(point, H1, H2);
	  if (!H1 && !H2) return Rot2::relativeBearing(d);
	  Matrix D_result_d;
	  Rot2 result = Rot2::relativeBearing(d, D_result_d);
	  if (H1) *H1 = D_result_d * (*H1);
	  if (H2) *H2 = D_result_d * (*H2);
	  return result;
  }

  /* ************************************************************************* */
  double Pose2::range(const Point2& point,
		  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (!H1 && !H2) return transform_to(point).norm();
	  Point2 d = transform_to(point, H1, H2);
	  double x = d.x(), y = d.y(), d2 = x * x + y * y, n = sqrt(d2);
	  Matrix D_result_d = Matrix_(1, 2, x / n, y / n);
	  if (H1) *H1 = D_result_d * (*H1);
	  if (H2) *H2 = D_result_d * (*H2);
	  return n;
  }

  /* ************************************************************************* */
} // namespace gtsam
