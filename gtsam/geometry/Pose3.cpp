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

#include <iostream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Lie-inl.h>

using namespace std;
using namespace boost::numeric::ublas;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Pose3);

  static const Matrix I3 = eye(3), _I3=-I3, I6 = eye(6), Z3 = zeros(3, 3);

  /* ************************************************************************* */
  // Calculate Adjoint map
  // Ad_pose is 6*6 matrix that when applied to twist xi, returns Ad_pose(xi)
  // Experimental - unit tests of derivatives based on it do not check out yet
  Matrix Pose3::AdjointMap() const {
		const Matrix R = R_.matrix();
		const Vector t = t_.vector();
		Matrix A = skewSymmetric(t)*R;
		Matrix DR = collect(2, &R, &Z3);
		Matrix Dt = collect(2, &A, &R);
		return gtsam::stack(2, &DR, &Dt);
	}

  /* ************************************************************************* */
  void Pose3::print(const string& s) const {
    R_.print(s + ".R");
    t_.print(s + ".t");
  }

  /* ************************************************************************* */
  bool Pose3::equals(const Pose3& pose, double tol) const {
    return R_.equals(pose.R_,tol) && t_.equals(pose.t_,tol);
  }

  /* ************************************************************************* */
  /** Modified from Murray94book version (which assumes w and v normalized?) */
  Pose3 Pose3::ExpmapFull(const Vector& xi) {

  	// get angular velocity omega and translational velocity v from twist xi
  	Point3 w(xi(0),xi(1),xi(2)), v(xi(3),xi(4),xi(5));

    double theta = w.norm();
		if (theta < 1e-10) {
		  static const Rot3 I;
			return Pose3(I, v);
		}
		else {
			Point3 n(w/theta); // axis unit vector
			Rot3 R = Rot3::rodriguez(n.vector(),theta);
			double vn = n.dot(v); // translation parallel to n
			Point3 n_cross_v = n.cross(v); // points towards axis
			Point3 t = (n_cross_v - R*n_cross_v)/theta + vn*n;
			return Pose3(R, t);
		}
  }

  /* ************************************************************************* */
  Vector Pose3::LogmapFull(const Pose3& p) {
    Vector w = Rot3::Logmap(p.rotation()), T = p.translation().vector();
  	double t = norm_2(w);
		if (t < 1e-10)
	    return concatVectors(2, &w, &T);
		else {
			Matrix W = skewSymmetric(w/t);
			Matrix Ainv = I3 - (0.5*t)*W + ((2*sin(t)-t*(1+cos(t)))/(2*sin(t))) * (W * W);
			Vector u = Ainv*T;
	    return concatVectors(2, &w, &u);
		}
  }

#ifdef CORRECT_POSE3_EXMAP
  /* ************************************************************************* */
  // Changes default to use the full verions of expmap/logmap
  /* ************************************************************************* */
  Pose3 Expmap(const Vector& xi) {
  	return Pose3::ExpmapFull(xi);
  }

  /* ************************************************************************* */
  Vector Logmap(const Pose3& p) {
  	return Pose3::LogmapFull(p);
  }

  /* ************************************************************************* */
  Pose3 expmap(const Vector& d) {
  	return expmapFull(d);
  }

  /* ************************************************************************* */
  Vector logmap(const Pose3& T1, const Pose3& T2) {
  	return logmapFull(T2);
  }

#else

  /* ************************************************************************* */
  /* incorrect versions for which we know how to compute derivatives */
  Pose3 Pose3::Expmap(const Vector& d) {
    Vector w = sub(d, 0,3);
    Vector u = sub(d, 3,6);
    return Pose3(Rot3::Expmap(w), Point3::Expmap(u));
  }

  /* ************************************************************************* */
  // Log map at identity - return the translation and canonical rotation
  // coordinates of a pose.
  Vector Pose3::Logmap(const Pose3& p) {
    const Vector w = Rot3::Logmap(p.rotation()), u = Point3::Logmap(p.translation());
    return concatVectors(2, &w, &u);
  }

  /** These are the "old-style" expmap and logmap about the specified
   * pose. Increments the offset and rotation independently given a translation and
   * canonical rotation coordinates. Created to match ML derivatives, but
   * superseded by the correct exponential map story in .cpp */
  Pose3 Pose3::expmap(const Vector& d) const {
    return Pose3(R_.expmap(sub(d, 0, 3)),
    		t_.expmap(sub(d, 3, 6)));
  }

  /** Independently computes the logmap of the translation and rotation. */
  Vector Pose3::logmap(const Pose3& pp) const {
    const Vector r(R_.logmap(pp.rotation())),
        t(t_.logmap(pp.translation()));
    return concatVectors(2, &r, &t);
  }

  #endif

  /* ************************************************************************* */
  Matrix Pose3::matrix() const {
    const Matrix R = R_.matrix(), T = Matrix_(3,1, t_.vector());
    const Matrix A34 = collect(2, &R, &T);
    const Matrix A14 = Matrix_(1,4, 0.0, 0.0, 0.0, 1.0);
    return gtsam::stack(2, &A34, &A14);
  }

  /* ************************************************************************* */
  Pose3 Pose3::transform_to(const Pose3& pose) const {
		Rot3 cRv = R_ * Rot3(pose.R_.inverse());
		Point3 t = pose.transform_to(t_);
		return Pose3(cRv, t);
	}

  /* ************************************************************************* */
  Point3 Pose3::transform_from(const Point3& p,
		  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) {
#ifdef CORRECT_POSE3_EXMAP
			const Matrix R = R_.matrix();
			Matrix DR = R*skewSymmetric(-p.x(), -p.y(), -p.z());
			*H1 = collect(2,&DR,&R);
#else
			Matrix DR;
			R_.rotate(p, DR, boost::none);
			*H1 = collect(2,&DR,&I3);
#endif
	  }
	  if (H2) *H2 = R_.matrix();
	  return R_ * p + t_;
  }

  /* ************************************************************************* */
  Point3 Pose3::transform_to(const Point3& p,
    		  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  const Point3 result = R_.unrotate(p - t_);
	  if (H1) { 
		const Point3& q = result;
		Matrix DR = skewSymmetric(q.x(), q.y(), q.z());
#ifdef CORRECT_POSE3_EXMAP
		*H1 =  collect(2, &DR, &_I3);
#else
		Matrix DT = - R_.transpose(); // negative because of sub
		*H1 =  collect(2,&DR,&DT);
#endif
	  }

	  if (H2) *H2 = R_.transpose();
	  return result;
  }

  /* ************************************************************************* */
  Pose3 Pose3::compose(const Pose3& p2,
		  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) {
#ifdef CORRECT_POSE3_EXMAP
		*H1 = AdjointMap(inverse(p2));
#else
		const Rot3& R2 = p2.rotation();
		const Point3& t2 = p2.translation();
		Matrix DR_R1 = R2.transpose(), DR_t1 = Z3;
		Matrix DR = collect(2, &DR_R1, &DR_t1);
		Matrix Dt;
		transform_from(t2, Dt, boost::none);
		*H1 = gtsam::stack(2, &DR, &Dt);
#endif
	  }
	  if (H2) {
#ifdef CORRECT_POSE3_EXMAP

		*H2 = I6;
#else
		Matrix R1 = rotation().matrix();
		Matrix DR = collect(2, &I3, &Z3);
		Matrix Dt = collect(2, &Z3, &R1);
		*H2 = gtsam::stack(2, &DR, &Dt);
#endif
	  }
	  return (*this) * p2;
  }

  /* ************************************************************************* */
  Pose3 Pose3::inverse(boost::optional<Matrix&> H1) const {
	  if (H1)
#ifdef CORRECT_POSE3_EXMAP
		{ *H1 = - AdjointMap(p); }
#else
	  {
		Matrix Rt = R_.transpose();
		Matrix DR_R1 = -R_.matrix(), DR_t1 = Z3;
		Matrix Dt_R1 = -skewSymmetric(R_.unrotate(t_).vector()), Dt_t1 = -Rt;
		Matrix DR = collect(2, &DR_R1, &DR_t1);
		Matrix Dt = collect(2, &Dt_R1, &Dt_t1);
		*H1 = gtsam::stack(2, &DR, &Dt);
	  }
#endif
      Rot3 Rt = R_.inverse();
      return Pose3(Rt, Rt*(-t_));
	}

  /* ************************************************************************* */
  // between = compose(p2,inverse(p1));
  Pose3 Pose3::between(const Pose3& p2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) const {
	  	Matrix invH;
		Pose3 invp1 = inverse(invH);
		Matrix composeH1;
		Pose3 result = invp1.compose(p2, composeH1, H2);
		if (H1) *H1 = composeH1 * invH;
		return result;
  }

  /* ************************************************************************* */
  double Pose3::range(const Point3& point,
		  boost::optional<Matrix&> H1,
		  boost::optional<Matrix&> H2) const {
	  if (!H1 && !H2) return transform_to(point).norm();
	  Point3 d = transform_to(point, H1, H2);
	  double x = d.x(), y = d.y(), z = d.z(),
			 d2 = x * x + y * y + z * z, n = sqrt(d2);
	  Matrix D_result_d = Matrix_(1, 3, x / n, y / n, z / n);
	  if (H1) *H1 = D_result_d * (*H1);
	  if (H2) *H2 = D_result_d * (*H2);
	  return n;
  }
} // namespace gtsam
