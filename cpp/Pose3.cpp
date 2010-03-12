/**
 * @file  Pose3.cpp
 * @brief 3D Pose
 */

#include <iostream>
#include "Pose3.h"
#include "Lie-inl.h"
#include "LieConfig.h"

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
  Matrix AdjointMap(const Pose3& p) {
		const Matrix R = p.rotation().matrix();
		const Vector t = p.translation().vector();
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

#ifdef CORRECT_POSE3_EXMAP

  /** Modified from Murray94book version (which assumes w and v normalized?) */
  template<> Pose3 expmap(const Vector& xi) {

  	// get angular velocity omega and translational velocity v from twist xi
  	Point3 w(xi(0),xi(1),xi(2)), v(xi(3),xi(4),xi(5));

    double theta = norm(w);
		if (theta < 1e-5) {
		  static const Rot3 I;
			return Pose3(I, v);
		}
		else {
			Point3 n(w/theta); // axis unit vector
			Rot3 R = rodriguez(n.vector(),theta);
			double vn = dot(n,v); // translation parallel to n
			Point3 n_cross_v = cross(n,v); // points towards axis
			Point3 t = (n_cross_v - R*n_cross_v)/theta + vn*n;
			return Pose3(R, t);
		}
  }

  Vector logmap(const Pose3& p) {
    Vector w = logmap(p.rotation()), T = p.translation().vector();
  	double t = norm_2(w);
		if (t < 1e-5)
	    return concatVectors(2, &w, &T);
		else {
			Matrix W = skewSymmetric(w/t);
			Matrix Ainv = I3 - (0.5*t)*W + ((2*sin(t)-t*(1+cos(t)))/(2*sin(t))) * (W * W);
			Vector u = Ainv*T;
	    return concatVectors(2, &w, &u);
		}
  }

  Pose3 expmap(const Pose3& T, const Vector& d) {
    return compose(T,expmap<Pose3>(d));
  }

  Vector logmap(const Pose3& T1, const Pose3& T2) {
    return logmap(between(T1,T2));
  }

#else

  /* incorrect versions for which we know how to compute derivatives */
  template<> Pose3 expmap(const Vector& d) {
    Vector w = sub(d, 0,3);
    Vector u = sub(d, 3,6);
    return Pose3(expmap<Rot3> (w), expmap<Point3> (u));
  }

  // Log map at identity - return the translation and canonical rotation
  // coordinates of a pose.
  Vector logmap(const Pose3& p) {
    const Vector w = logmap(p.rotation()), u = logmap(p.translation());
    return concatVectors(2, &w, &u);
  }

  /** These are the "old-style" expmap and logmap about the specified
   * pose. Increments the offset and rotation independently given a translation and
   * canonical rotation coordinates. Created to match ML derivatives, but
   * superseded by the correct exponential map story in .cpp */
  Pose3 expmap(const Pose3& p0, const Vector& d) {
    return Pose3(expmap(p0.rotation(), sub(d, 0, 3)),
        expmap(p0.translation(), sub(d, 3, 6)));
  }

  /** Independently computes the logmap of the translation and rotation. */
  Vector logmap(const Pose3& p0, const Pose3& pp) {
    const Vector r(logmap(p0.rotation(), pp.rotation())),
        t(logmap(p0.translation(), pp.translation()));
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
		Rot3 cRv = R_ * Rot3(gtsam::inverse(pose.R_));
		Point3 t = gtsam::transform_to(pose, t_);
		return Pose3(cRv, t);
	}

  /* ************************************************************************* */
  Point3 transform_from(const Pose3& pose, const Point3& p) {
    return pose.rotation() * p + pose.translation();
  }

  /* ************************************************************************* */
  Matrix Dtransform_from1(const Pose3& pose, const Point3& p) {
#ifdef CORRECT_POSE3_EXMAP
		const Matrix R = pose.rotation().matrix();
    Matrix DR = R*skewSymmetric(-p.x(), -p.y(), -p.z());
    return collect(2,&DR,&R);
#else
    Matrix DR = Drotate1(pose.rotation(), p);
    return collect(2,&DR,&I3);
#endif
  }

  /* ************************************************************************* */
  Matrix Dtransform_from2(const Pose3& pose) {
    return pose.rotation().matrix();
  }

  /* ************************************************************************* */
  Point3 transform_to(const Pose3& pose, const Point3& p) {
    Point3 sub = p - pose.translation();
    return unrotate(pose.rotation(), sub);
  }

  /* ************************************************************************* */
  // see math.lyx, derivative of action
  Matrix Dtransform_to1(const Pose3& pose, const Point3& p) {
    Point3 q = transform_to(pose,p);
    Matrix DR = skewSymmetric(q.x(), q.y(), q.z());
#ifdef CORRECT_POSE3_EXMAP
		return collect(2, &DR, &_I3);
#else
    Matrix DT = - pose.rotation().transpose(); // negative because of sub
    return collect(2,&DR,&DT);
#endif
    }

  /* ************************************************************************* */
  Matrix Dtransform_to2(const Pose3& pose, const Point3& p) {
    return pose.rotation().transpose();
  }

  /* ************************************************************************* */
  // compose = Pose3(compose(R1,R2),transform_from(p1,t2)
  Matrix Dcompose1(const Pose3& p1, const Pose3& p2) {
#ifdef CORRECT_POSE3_EXMAP
		return AdjointMap(inverse(p2));
#else
		const Rot3& R2 = p2.rotation();
		const Point3& t2 = p2.translation();
  	Matrix DR_R1 = R2.transpose(), DR_t1 = Z3;
		Matrix DR = collect(2, &DR_R1, &DR_t1);
		Matrix Dt = Dtransform_from1(p1, t2);
		return gtsam::stack(2, &DR, &Dt);
#endif
	}

	Matrix Dcompose2(const Pose3& p1, const Pose3& p2) {
#ifdef CORRECT_POSE3_EXMAP

		return I6;
#else
		Matrix R1 = p1.rotation().matrix();
		Matrix DR = collect(2, &I3, &Z3);
		Matrix Dt = collect(2, &Z3, &R1);
		return gtsam::stack(2, &DR, &Dt);
#endif
	}

  /* ************************************************************************* */
  // inverse = Pose3(inverse(R),-unrotate(R,t));
	// TODO: combined function will save !
	Matrix Dinverse(const Pose3& p) {
#ifdef CORRECT_POSE3_EXMAP
		return - AdjointMap(p);
#else
		const Rot3& R = p.rotation();
		const Point3& t = p.translation();
		Matrix Rt = R.transpose();
		Matrix DR_R1 = -R.matrix(), DR_t1 = Z3;
		Matrix Dt_R1 = -skewSymmetric(unrotate(R,t).vector()), Dt_t1 = -Rt;
		Matrix DR = collect(2, &DR_R1, &DR_t1);
		Matrix Dt = collect(2, &Dt_R1, &Dt_t1);
		return gtsam::stack(2, &DR, &Dt);
#endif
	}

  /* ************************************************************************* */
  // between = compose(p2,inverse(p1));
  Pose3 between(const Pose3& p1, const Pose3& p2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) {
		Pose3 invp1 = inverse(p1);
		Pose3 result = compose(invp1, p2);
		if (H1) *H1 = Dcompose1(invp1, p2) * Dinverse(p1);
		if (H2) *H2 = Dcompose2(invp1, p2);
		return result;
	}

  /* ************************************************************************* */
} // namespace gtsam
